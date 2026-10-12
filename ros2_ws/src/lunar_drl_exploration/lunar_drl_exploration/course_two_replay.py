"""Phase/episode quotas for the explicitly versioned course-two trial.

Only training metadata reads reference masks. Nothing here filters policy actions.
The index holds small IDs, not additional observation or truth rasters.
"""
from collections import Counter, defaultdict, deque
import math

import numpy as np

OLD = ('moon_inside', 'cave_inside', 'moon_outside', 'cave_outside')
QUOTAS = {**dict.fromkeys(OLD, 8), 'new_cover': 16, 'new_approach': 8,
          'success_finish': 4, 'new_finish': 4}
BITS = np.array([i.bit_count() for i in range(256)], dtype=np.uint8)


def cell_counts(state, scenes):
    reference = scenes[state.scene_id].packed_reference
    return int(BITS[state.observed & reference].sum()), int(BITS[reference].sum())


class PhaseIndex:
    def __init__(self, completion_bonus=20., task_scope=''):
        if not math.isfinite(completion_bonus) or completion_bonus <= 0:
            raise ValueError('positive completion bonus required for success proof')
        self.completion_bonus = completion_bonus
        if task_scope not in ('','whole_cave_v1'):raise ValueError('unknown replay task scope')
        self.task_scope = task_scope
        self.sampling = ''
        self.scope_excluded = set()
        self.rows = {}
        self.pools = defaultdict(lambda: defaultdict(set))
        self.ends = {}
        self._status = None
        self.last_batch = {}

    @property
    def bytes_used(self):
        # Conservative allowance for ID dictionaries/sets and cached reports.
        # Transition payloads are owned and charged once by ReplayBuffer.
        return 2048*len(self.rows) + 1024*sum(len(p) for p in self.pools.values()) + 128*len(self.scope_excluded)

    @staticmethod
    def classify(transition, scenes):
        task = scenes[transition.privileged.scene_id].generator_descriptor.get('curriculum_task')
        if task in OLD:
            return task
        if not isinstance(task, str) or not task.startswith('c2_'):
            raise ValueError('unclassified course-two replay task')
        covered, total = cell_counts(transition.privileged, scenes)
        if total <= 0:
            raise ValueError('empty course-two coverage reference')
        if covered == 0:
            return 'new_approach'
        if covered*100 < total*90:
            return 'new_cover'
        if covered*100 < total*99:
            return 'new_finish'
        raise ValueError('course-two transition begins after completion')

    def add(self, index, transition, scenes):
        if index in self.rows or index in self.scope_excluded:
            raise ValueError('duplicate balanced replay index')
        desc=scenes[transition.privileged.scene_id].generator_descriptor
        cave=desc.get('family')=='cave' or str(desc.get('curriculum_task','')).startswith('cave_')
        if self.task_scope and cave and desc.get('task_scope')!=self.task_scope:
            self.scope_excluded.add(index);self._status=None
            return
        phase = self.classify(transition, scenes)
        episode = transition.episode_id
        self.rows[index] = (episode, phase, transition.step_index)
        self.pools[phase][episode].add(index)
        if phase == 'new_finish' and transition.terminated and transition.step_index is not None:
            after, total = cell_counts(transition.next_privileged, scenes)
            p = transition.parts
            base = p.removed_frontiers_in/50 - .002*p.distance_m - .005*p.turn_rad/math.pi - .001
            if after*100 >= 99*total and math.isclose(transition.reward-base,
                    self.completion_bonus, rel_tol=0., abs_tol=1e-6):
                self.ends[episode] = (transition.step_index, index)
        self._status = None

    def remove(self, index):
        if index in self.scope_excluded:
            self.scope_excluded.remove(index);self._status=None
            return
        episode, phase, _ = self.rows.pop(index)
        pool = self.pools[phase]
        pool[episode].remove(index)
        if not pool[episode]:
            del pool[episode]
        if not pool:
            del self.pools[phase]
        if self.ends.get(episode, (None,None))[1] == index:
            del self.ends[episode]
        self._status = None

    @property
    def success_indices(self):
        result = set()
        for episode, (end, _) in self.ends.items():
            for index in self.pools.get('new_finish', {}).get(episode, ()):
                step = self.rows[index][2]
                if step is not None and end-15 <= step <= end:
                    result.add(index)
        return result

    def status(self):
        if self._status is None:
            counts = {phase:sum(map(len,pool.values())) for phase,pool in self.pools.items()}
            episodes = {phase:len(pool) for phase,pool in self.pools.items()}
            missing = [phase for phase in (*OLD,'new_cover','new_approach','new_finish')
                       if episodes.get(phase,0) < 4]
            successes = {self.rows[k][0] for k in self.success_indices}
            if len(successes) < 2:
                missing.append('success_finish')
            if not missing:
                try:
                    self._allocate(np.random.default_rng(0))
                except ValueError:
                    missing.append('global_batch_capacity')
            self._status = dict(ready=not missing, missing=missing, pool_counts=counts,
                task_scope=self.task_scope,scope_excluded_transitions=len(self.scope_excluded),
                episode_counts=episodes, successful_finish_episodes=len(successes),
                successful_finish_transitions=len(self.success_indices),
                phase_quotas={**dict.fromkeys(OLD,8),'new_cover':16,'new_approach':8,'new_finish':8})
        # Caller cannot change the cached readiness decision.
        return {k:dict(v) if isinstance(v,dict) else list(v) if isinstance(v,list) else v
                for k,v in self._status.items()}

    def _allocate(self, rng):
        """Exact quota flow: bucket -> transition (1) -> episode (4).

        Episode order is shuffled before transitions, so long episodes do not
        receive more initial tickets. A shared transition node deduplicates the
        overlapping successful/general finish pools. Flow prevents greedy false
        shortages when an episode contributes to multiple phases.
        """
        graph = [[], []]
        nodes = {}
        def node(key):
            if key not in nodes:
                nodes[key] = len(graph)
                graph.append([])
            return nodes[key]
        def edge(a,b,capacity):
            forward = [b,len(graph[b]),capacity]
            graph[a].append(forward)
            graph[b].append([a,len(graph[a])-1,0])
            return forward
        seen = set()
        episode_nodes = set()
        choices = []
        success = self.success_indices
        for phase in rng.permutation(list(QUOTAS)):
            bucket = node(('bucket',phase))
            edge(0,bucket,QUOTAS[phase])
            pool = self.pools.get('new_finish' if phase=='success_finish' else phase, {})
            for episode in rng.permutation(sorted(pool)):
                eligible = sorted(pool[episode] & success if phase=='success_finish' else pool[episode])
                for index in rng.permutation(eligible):
                    index = int(index)
                    item = node(('item',index))
                    choices.append((phase,index,edge(bucket,item,1)))
                    if index not in seen:
                        ep = node(('episode',episode))
                        edge(item,ep,1)
                        if episode not in episode_nodes:
                            edge(ep,1,4)
                            episode_nodes.add(episode)
                        seen.add(index)
        flow = 0
        while flow < 64:
            level = [-1]*len(graph)
            level[0] = 0
            queue = deque([0])
            while queue:
                a = queue.popleft()
                for b,_,cap in graph[a]:
                    if cap and level[b] < 0:
                        level[b] = level[a]+1
                        queue.append(b)
            if level[1] < 0:
                raise ValueError('insufficient global bucket/episode supply for 64 distinct samples')
            cursor = [0]*len(graph)
            def send(a, amount):
                if a == 1:
                    return amount
                while cursor[a] < len(graph[a]):
                    e = graph[a][cursor[a]]
                    b,reverse,cap = e
                    if cap and level[b] == level[a]+1:
                        sent = send(b,min(amount,cap))
                        if sent:
                            e[2] -= sent
                            graph[b][reverse][2] += sent
                            return sent
                    cursor[a] += 1
                return 0
            while flow < 64:
                sent = send(0,64-flow)
                if not sent:
                    break
                flow += sent
        selected = [(phase,index) for phase,index,e in choices if e[2]==0]
        if len(selected)!=64 or len({i for _,i in selected})!=64:
            raise RuntimeError('balanced batch allocation lost deduplication')
        return selected

    def sample_indices(self, rng):
        status = self.status()
        if not status['ready']:
            raise ValueError('missing course-two replay buckets: '+', '.join(status['missing']))
        selected = self._allocate(rng)
        indices = [index for _,index in selected]
        rng.shuffle(indices)
        self.last_batch = dict(roles=dict(Counter(phase for phase,_ in selected)),
            phases=dict(Counter(self.rows[k][1] for k in indices)),
            unique_transitions=len(set(indices)), episodes=len({self.rows[k][0] for k in indices}),
            max_per_episode=max(Counter(self.rows[k][0] for k in indices).values()))
        return indices

    def required_indices(self):
        """Pin diverse anchors and their terminal proof across bounded saves."""
        required = set()
        for phase in sorted(self.pools):
            pool = self.pools[phase]
            for episode in sorted(pool)[:4]:
                required.update(sorted(pool[episode])[-4:])
                if episode in self.ends:
                    required.add(self.ends[episode][1])
        successful = sorted({self.rows[k][0] for k in self.success_indices})[:2]
        for episode in successful:
            required.add(self.ends[episode][1])
            required.update(sorted(self.pools['new_finish'][episode] & self.success_indices)[-4:])
        if self.status()['ready']:
            required.update(k for _,k in self._allocate(np.random.default_rng(0)))
            # Any successful-window anchor selected by flow keeps its proof.
            for index in tuple(required):
                episode = self.rows[index][0]
                if episode in self.ends:
                    required.add(self.ends[episode][1])
        return required
