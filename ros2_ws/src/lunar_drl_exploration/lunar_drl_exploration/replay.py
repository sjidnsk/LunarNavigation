"""Bounded immutable replay and explicit primitive transport (trusted local peers)."""
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
import math
import io
import pickle
import sys
from types import MappingProxyType

import numpy as np
from .metrics import ResourceLimitError
from .feature_schema import (PRIVILEGED_SCHEMA, frontier_semantics,
                             require_frontier_semantics, require_observation_identity)
from .contracts import (DecisionObservation, PrivilegedScene, PrivilegedState, PrivilegedActionContext,
                        Transition, RewardParts, Pose, TaskSpec, SensorSpec)

_TYPES = {cls.__name__: cls for cls in (DecisionObservation, PrivilegedScene,
          PrivilegedState, PrivilegedActionContext, Transition, RewardParts, Pose, TaskSpec, SensorSpec)}


_SNAPSHOT_MAGIC = b'LDRLRP3\0'
_SNAPSHOT_HEADER_BYTES = len(_SNAPSHOT_MAGIC) + 8


def snapshot_sample_count(payload):
    """O(1) envelope check only; actual payload/count/reference validation is on restore."""
    if (not isinstance(payload, bytes) or len(payload) <= _SNAPSHOT_HEADER_BYTES or
            not payload.startswith(_SNAPSHOT_MAGIC)):
        raise ValueError('replay snapshot envelope schema mismatch')
    return int.from_bytes(payload[len(_SNAPSHOT_MAGIC):_SNAPSHOT_HEADER_BYTES], 'little')


def _backing(array):
    base = array
    while isinstance(base, np.ndarray) and base.base is not None:
        base = base.base
    if isinstance(base, memoryview):
        base = base.obj
    return base


def dumps_transport(value, *, _prefix=b''):
    """Encode whitelisted contracts/containers as primitives, preserving aliases.

    Maps/native terrain are intentionally unsupported. Workers keep maps locally.
    NumPy buffers become bytes; no NumPy/dataclass/mappingproxy pickle reducer is used.
    """
    nodes, memo, owners = [], {}, {}

    def encode(item):
        if item is None or type(item) in (str, bytes, bool, int, float):
            return ('literal', item)
        if isinstance(item, np.generic):
            return encode(item.item())
        key = id(item)
        if key in memo:
            return ('ref', memo[key])
        index = len(nodes); memo[key] = index; nodes.append(None)
        if isinstance(item, np.ndarray):
            if item.dtype.hasobject:
                raise TypeError('object arrays cannot be transported')
            owner = _backing(item)
            if id(owner) not in owners:
                if isinstance(owner, np.ndarray):
                    if not owner.flags.c_contiguous:
                        raise TypeError('array backing must be contiguous')
                    data = owner.tobytes()
                    address = owner.ctypes.data
                else:
                    data = bytes(owner)
                    address = np.frombuffer(owner, dtype=np.uint8).ctypes.data
                owners[id(owner)] = (data, address)
            data, address = owners[id(owner)]
            node = ('array', item.dtype.str, item.shape, item.strides,
                    item.ctypes.data - address, data)
        elif type(item).__name__ in _TYPES and type(item) is _TYPES[type(item).__name__]:
            node = ('contract', type(item).__name__,
                    {f.name: encode(getattr(item, f.name)) for f in fields(item)})
        elif isinstance(item, Mapping):
            node = ('frozen_mapping' if isinstance(item, MappingProxyType) else 'mapping',
                    [(encode(k), encode(v)) for k, v in item.items()])
        elif isinstance(item, (tuple, list)):
            node = ('tuple' if isinstance(item, tuple) else 'list', [encode(v) for v in item])
        else:
            raise TypeError(f'unsupported transport type: {type(item).__name__}')
        nodes[index] = node
        return ('ref', index)

    root = encode(value)
    record = dict(schema='immutable_transport_v1', frontier_semantics=frontier_semantics(),
                  root=root, nodes=nodes)
    if not _prefix:
        return pickle.dumps(record, protocol=5)
    # One output stream: adding the fixed replay envelope never concatenates a
    # second multi-GiB payload. BytesIO.getvalue owns its immutable result.
    with io.BytesIO() as stream:
        stream.write(_prefix)
        pickle.dump(record, stream, protocol=5)
        return stream.getvalue()


def loads_transport(payload):
    """Trusted local transport only; reconstruct immutable arrays after real IPC/load."""
    record = pickle.loads(payload)
    if record.get('schema') != 'immutable_transport_v1':
        raise ValueError('transport schema mismatch')
    require_frontier_semantics(record)
    memo = {}

    def decode(ref):
        kind, value = ref
        if kind == 'literal':
            return value
        if kind != 'ref':
            raise ValueError('invalid transport reference')
        if value in memo:
            return memo[value]
        node = record['nodes'][value]; tag = node[0]
        if tag == 'array':
            _, dtype, shape, strides, offset, data = node
            dtype = np.dtype(dtype)
            if dtype.hasobject or not isinstance(data, bytes):
                raise ValueError('invalid numeric backing')
            result = np.ndarray(shape, dtype=dtype, buffer=data, offset=offset, strides=strides)
        elif tag == 'contract':
            cls = _TYPES.get(node[1])
            if cls is None:
                raise ValueError('unknown transport contract')
            kwargs = {k: decode(v) for k, v in node[2].items()}
            if cls in (DecisionObservation, PrivilegedState, PrivilegedActionContext) and 'schema' not in kwargs:
                raise ValueError('transport contract schema is required')
            try:
                result = cls(**kwargs)  # Run normal contract validation.
            except TypeError as error:
                raise ValueError('incompatible transport contract fields') from error
            # Validation copies arrays; restore already immutable decoded views to
            # retain graph and packed-mask sharing across adjacent transitions.
            for key, item in kwargs.items():
                if isinstance(item, np.ndarray):
                    validated = getattr(result, key)
                    if validated.dtype != item.dtype or validated.shape != item.shape:
                        raise ValueError('noncanonical contract array')
                    object.__setattr__(result, key, item)
        elif tag in ('mapping', 'frozen_mapping'):
            result = {decode(k): decode(v) for k, v in node[1]}
            if tag == 'frozen_mapping': result = MappingProxyType(result)
        elif tag in ('tuple', 'list'):
            result = [decode(v) for v in node[1]]
            if tag == 'tuple': result = tuple(result)
        else:
            raise ValueError('unknown transport node')
        memo[value] = result
        return result

    return decode(record['root'])


def _objects(root):
    """Unique reachable Python allocations; array storage is counted only at owner."""
    pending, found = [root], {}
    while pending:
        value = pending.pop(); key = id(value)
        if key in found:
            continue
        size = sys.getsizeof(value)
        if isinstance(value, np.ndarray):
            if not value.flags.owndata and value.base is not None:
                pending.append(value.base)
        elif is_dataclass(value):
            pending.extend(getattr(value, f.name) for f in fields(value))
            size += sys.getsizeof(value.__dict__)
        elif isinstance(value, Mapping):
            for k, v in value.items(): pending.extend((k, v))
            if isinstance(value, MappingProxyType): size += sys.getsizeof(dict(value))
        elif isinstance(value, (list, tuple)):
            pending.extend(value)
        elif isinstance(value, memoryview):
            pending.append(value.obj)
        # Conservative unique ledger-row/key/refcount overhead. Per-root
        # membership tuples and their fresh ID integers are charged separately.
        found[key] = (value, size + 192)
    return found


def _same_content(left, right):
    """Exact immutable boundary equality, including separately decoded arrays."""
    if left is right:
        return True
    if type(left) is not type(right):
        return False
    if isinstance(left, np.ndarray):
        return left.dtype == right.dtype and np.array_equal(left, right)
    if is_dataclass(left):
        return all(_same_content(getattr(left, field.name), getattr(right, field.name))
                   for field in fields(left))
    if isinstance(left, Mapping):
        return (left.keys() == right.keys() and
                all(_same_content(value, right[key]) for key, value in left.items()))
    if isinstance(left, (tuple, list)):
        return len(left) == len(right) and all(_same_content(a, b) for a, b in zip(left, right))
    return bool(left == right)


def transition_task(transition, scenes):
    """Frozen episode task; robot movement never changes this replay label."""
    labels = []
    for state in (transition.privileged, transition.next_privileged):
        truth = scenes.get(state.scene_id)
        if not isinstance(truth, PrivilegedScene):
            raise ValueError(f'missing static scene for task: {state.scene_id}')
        task = truth.generator_descriptor.get('curriculum_task', 'unclassified')
        if task is None:
            task = 'unclassified'  # Explicit None is the normal non-course reset.
        if not isinstance(task, str) or not task:
            raise ValueError('nonempty curriculum task label required')
        labels.append(task)
    if labels[0] != labels[1]:
        raise ValueError('transition cannot cross curriculum task labels')
    return labels[0]


class _Ledger:
    def __init__(self):
        self.items = {}; self.bytes = 0; self.buffer_bytes = 0
        self.membership_bytes = 0

    def acquire(self, root):
        objects = _objects(root)
        for key, (value, size) in objects.items():
            if key in self.items:
                self.items[key][2] += 1
            else:
                self.items[key] = [value, size, 1]; self.bytes += size
                if isinstance(value, bytes): self.buffer_bytes += len(value)
        keys = tuple(objects)
        self.membership_bytes += self._membership_size(keys)
        return keys

    @staticmethod
    def _membership_size(keys):
        # id() makes fresh Python integers even when their payload object is shared.
        return sys.getsizeof(keys) + sum(sys.getsizeof(key) for key in keys)

    def release(self, keys):
        self.membership_bytes -= self._membership_size(keys)
        for key in keys:
            entry = self.items[key]; entry[2] -= 1
            if entry[2] == 0:
                self.bytes -= entry[1]
                if isinstance(entry[0], bytes): self.buffer_bytes -= len(entry[0])
                del self.items[key]
        if not self.items: self.items = {}


class ReplayBuffer:
    def __init__(self, max_bytes=8 * 1024**3):
        if int(max_bytes) != max_bytes or max_bytes <= 0:
            raise ValueError('positive replay byte budget required')
        self.max_bytes = int(max_bytes)
        self._entries = {}; self._first = self._next = 0; self._scenes = {}; self._refs = {}; self._scene_keys = {}
        self._sequence_index = {}
        self._task_entries = {}; self._task_positions = {}; self._task_bytes = 0
        self._ledger = _Ledger()
        self._entry_bytes = self._refcount_bytes = self._sequence_bytes = 0

    @property
    def bytes_used(self):
        if not self._entries:
            return 0  # Empty baseline dictionaries retain no entry capacity.
        tables = sum(sys.getsizeof(table) for table in
                     (self._entries, self._scenes, self._refs,
                      self._scene_keys, self._sequence_index, self._ledger.items))
        tables += sys.getsizeof(self._task_entries) + sys.getsizeof(self._task_positions)
        tables += sum(sys.getsizeof(task) + sys.getsizeof(entries)
                      for task, entries in self._task_entries.items())
        return (self._ledger.bytes + self._ledger.membership_bytes + tables +
                self._entry_bytes + self._refcount_bytes + self._sequence_bytes + self._task_bytes)

    @property
    def buffer_bytes(self):
        return self._ledger.buffer_bytes

    @property
    def scenes(self):
        return MappingProxyType(self._scenes)

    @property
    def scene_refcounts(self):
        return dict(self._refs)

    @property
    def task_counts(self):
        return {task: len(entries) for task, entries in self._task_entries.items()}

    def __len__(self):
        return len(self._entries)

    def add(self, transition, scenes):
        if not isinstance(transition, Transition):
            raise TypeError('only finished Transition records enter replay')
        for obs in (transition.observation, transition.next_observation):
            require_observation_identity(obs)
        for state in (transition.privileged, transition.next_privileged):
            if state.schema != PRIVILEGED_SCHEMA or (state.actions is not None and
                    state.actions.schema != PRIVILEGED_SCHEMA):
                raise ValueError('incompatible privileged frontier schema')
        if (transition.step_index is not None and
                (transition.episode_id, transition.step_index) in self._sequence_index):
            raise ValueError('duplicate episode step in replay')
        if not (math.isfinite(transition.reward) and
                0 <= transition.action < len(transition.observation.goals)):
            raise ValueError('invalid finished transition')
        if not transition.terminated and not len(transition.next_observation.goals):
            raise ValueError('nonterminal successor needs actions')
        ids = {transition.privileged.scene_id, transition.next_privileged.scene_id}
        selected = {}
        for key in ids:
            truth = self._scenes.get(key, scenes.get(key))
            if not isinstance(truth, PrivilegedScene) or truth.scene_id != key:
                raise ValueError(f'missing or invalid static scene: {key}')
            selected[key] = truth
        transition_task(transition, selected)
        for state in (transition.privileged, transition.next_privileged):
            truth = selected[state.scene_id]
            if state.observed.shape != ((math.prod(truth.reference_shape)+7)//8,):
                raise ValueError('observed bits do not match scene reference')
        for observation, state in ((transition.observation, transition.privileged),
                                   (transition.next_observation, transition.next_privileged)):
            context = state.actions
            if context is None:
                raise ValueError('candidate action context required for replay admission')
            if len(context.action_positions) != len(observation.goals):
                raise ValueError('candidate action context must align with observation actions')
            if (len(observation.goals) and not np.array_equal(
                    context.positions[context.action_positions], observation.goals[:, :2])):
                raise ValueError('candidate positions must exactly match frozen action goals')
            if not np.array_equal(context.action_yaws, observation.goals[:, 2]):
                raise ValueError('candidate action yaws must exactly match frozen action goals')
            if np.any(context.support_indices >= len(selected[state.scene_id].positions)):
                raise ValueError('candidate support index is outside truth graph')
        # Validate one whole admission before evicting anything. Oversized complete
        # messages stay with the collector until the caller handles this failure.
        single = ReplayBuffer(self.max_bytes)
        single._first = single._next = self._next
        single._append(transition, selected, ids)
        if single.bytes_used > self.max_bytes:
            raise ValueError('complete transition and scenes exceed replay budget')
        self._append(transition, selected, ids)
        while self.bytes_used > self.max_bytes:
            if len(self._entries) == 1:
                # Eviction can leave a large table allocation behind. The exact
                # single-entry preflight already proved the fresh ownership fits.
                self.__dict__.update(single.__dict__)
                break
            self.evict_oldest()

    def _append(self, transition, selected, ids):
        for key, truth in selected.items():
            if key not in self._scenes:
                self._scenes[key] = truth; self._refs[key] = 0
                self._scene_keys[key] = self._ledger.acquire(truth)
                self._refcount_bytes += sys.getsizeof(0)
            self._refcount_bytes -= sys.getsizeof(self._refs[key])
            self._refs[key] += 1
            self._refcount_bytes += sys.getsizeof(self._refs[key])
        keys = self._ledger.acquire(transition)
        entry = (transition, ids, keys)
        self._entries[self._next] = entry
        task = transition_task(transition, selected)
        entries = self._task_entries.setdefault(task, [])
        position = len(entries)
        entries.append(self._next)
        self._task_positions[self._next] = position
        self._task_bytes += 2 * sys.getsizeof(self._next) + sys.getsizeof(position)
        self._entry_bytes += self._entry_size(self._next, entry)
        if transition.step_index is not None:
            key = (transition.episode_id, transition.step_index)
            self._sequence_index[key] = self._next
            self._sequence_bytes += self._sequence_size(key, self._next)
        self._next += 1

    @staticmethod
    def _entry_size(key, entry):
        # Membership tuple/ID integers live in the ledger's per-root charge.
        return sys.getsizeof(key) + sys.getsizeof(entry) + sys.getsizeof(entry[1])

    @staticmethod
    def _sequence_size(key, entry_id):
        # The key's episode/step objects already belong to the transition ledger.
        # Charge the new tuple and conservatively charge its entry-ID reference.
        return sys.getsizeof(key) + sys.getsizeof(entry_id)

    def evict_oldest(self):
        entry = self._entries.pop(self._first)
        self._entry_bytes -= self._entry_size(self._first, entry)
        transition, ids, keys = entry
        task = transition_task(transition, self._scenes)
        entries = self._task_entries[task]
        position = self._task_positions.pop(self._first)
        self._task_bytes -= 2 * sys.getsizeof(self._first) + sys.getsizeof(position)
        last = entries.pop()
        if position < len(entries):
            entries[position] = last
            self._task_bytes += sys.getsizeof(position) - sys.getsizeof(self._task_positions[last])
            self._task_positions[last] = position
        if not entries:
            del self._task_entries[task]
        if transition.step_index is not None:
            key = (transition.episode_id, transition.step_index)
            entry_id = self._sequence_index.pop(key)
            self._sequence_bytes -= self._sequence_size(key, entry_id)
        self._first += 1
        self._ledger.release(keys)
        for key in ids:
            self._refcount_bytes -= sys.getsizeof(self._refs[key])
            self._refs[key] -= 1
            if self._refs[key] == 0:
                self._ledger.release(self._scene_keys.pop(key))
                del self._refs[key]; del self._scenes[key]
            else:
                self._refcount_bytes += sys.getsizeof(self._refs[key])
        if not self._entries:
            self._entries = {}; self._scenes = {}; self._refs = {}; self._scene_keys = {}
            self._sequence_index = {}
            self._task_entries = {}; self._task_positions = {}; self._task_bytes = 0

    def terminal_records(self):
        """Read-only terminal records in retained chronological order."""
        for transition, _, _ in self._entries.values():
            if transition.terminated or transition.truncated:
                yield transition

    def sample(self, batch_size, rng):
        if not self._entries or batch_size <= 0:
            raise ValueError('nonempty replay and positive batch required')
        # Replacement is intentional; effective SAC batch stays 64 even when a
        # constrained restore retained fewer than 64 complete transitions.
        indices = rng.integers(0, len(self._entries), size=batch_size)
        return [self._entries[self._first + int(index)][0] for index in indices]

    def sample_stratified(self, batch_size, rng, weights):
        """Integer largest-remainder quotas, O(tasks + batch), with replacement."""
        if (isinstance(batch_size, (bool, np.bool_)) or
                not isinstance(batch_size, (int, np.integer)) or batch_size <= 0):
            raise ValueError('positive integer batch required')
        if not isinstance(weights, Mapping) or not weights:
            raise ValueError('nonempty task weights required')
        active = []
        for task, weight in weights.items():
            if (not isinstance(task, str) or isinstance(weight, (bool, np.bool_)) or
                    not isinstance(weight, (int, float, np.number)) or
                    not math.isfinite(weight) or weight < 0):
                raise ValueError('finite nonnegative task weights required')
            if weight > 0:
                if not self._task_entries.get(task):
                    raise ValueError(f'missing replay task: {task}')
                active.append((task, float(weight)))
        total = sum(weight for _, weight in active)
        if not active or not math.isfinite(total):
            raise ValueError('positive finite total task weight required')
        expected = [batch_size * weight / total for _, weight in active]
        quotas = [math.floor(value) for value in expected]
        # Rotate ties using the saved caller RNG, avoiding systematic old-task bias.
        tie_order = rng.permutation(len(active))
        ranked = sorted(tie_order, key=lambda i: expected[i] - quotas[i], reverse=True)
        for index in ranked[:batch_size - sum(quotas)]:
            quotas[index] += 1
        result = []
        for (task, _), quota in zip(active, quotas):
            entries = self._task_entries[task]
            for index in rng.integers(0, len(entries), size=quota):
                result.append(self._entries[entries[int(index)]][0])
        rng.shuffle(result)
        return result

    def sample_sequences(self, batch_size, rng, max_steps, weights=None):
        """Uniform original anchors followed by available same-episode steps.

        Returned tuples only reference stored immutable transitions. Index upkeep
        is independent of the requested horizon, so both training arms have the
        same replay ownership and byte-budget eviction rules.
        """
        if (isinstance(max_steps, (bool, np.bool_)) or
                not isinstance(max_steps, (int, np.integer)) or not 1 <= max_steps <= 8):
            raise ValueError('integer sequence horizon from 1 through 8 required')
        anchors = (self.sample(batch_size, rng) if weights is None else
                   self.sample_stratified(batch_size, rng, weights))
        result = []
        for anchor in anchors:
            sequence = [anchor]
            while len(sequence) < max_steps:
                previous = sequence[-1]
                if previous.step_index is None or previous.terminated or previous.truncated:
                    break
                key = (previous.episode_id, previous.step_index + 1)
                entry_id = self._sequence_index.get(key)
                if entry_id is None:
                    break
                following = self._entries[entry_id][0]
                if not (previous.privileged.scene_id == previous.next_privileged.scene_id ==
                        following.privileged.scene_id == following.next_privileged.scene_id):
                    break
                if not (_same_content(previous.next_observation, following.observation) and
                        _same_content(previous.next_privileged, following.privileged)):
                    break
                sequence.append(following)
            result.append(tuple(sequence))
        return result

    def snapshot(self, max_bytes=2 * 1024**3, *, require_sample=False):
        selected = ReplayBuffer(max(1, max_bytes))
        active_tasks = set(self._task_entries) - {'unclassified'}
        # Put one recent representative of each course task last. Bounded FIFO
        # trimming then retains every class, or explicitly rejects the snapshot.
        reserved = set()
        for task in active_tasks:
            # An unusually large recent transition must not mask a fitting old
            # representative from the same task.
            for index in sorted(self._task_entries[task], reverse=True):
                single = ReplayBuffer(max(1, max_bytes))
                try:
                    single.add(self._entries[index][0], self._scenes)
                except ValueError as error:
                    if 'exceed replay budget' not in str(error):
                        raise
                    continue
                reserved.add(index)
                break
            else:
                raise ResourceLimitError(
                    f'replay snapshot cannot preserve active task {task} within {max_bytes} bytes', 0, 1)
        order = [index for index in self._entries if index not in reserved]
        order.extend(sorted(reserved))
        source_indices = {}
        for index in order:
            transition = self._entries[index][0]
            try:
                selected.add(transition, self._scenes)
                source_indices[selected._next - 1] = index
            except ValueError as error:
                if 'exceed replay budget' not in str(error): raise
        while True:
            if not active_tasks.issubset(selected.task_counts):
                raise ResourceLimitError(
                    f'replay snapshot cannot preserve active task samples within {max_bytes} bytes',
                    len(active_tasks.intersection(selected.task_counts)), len(active_tasks))
            if require_sample and not len(selected):
                raise ResourceLimitError(
                    f'replay snapshot requires a complete sample within {max_bytes} bytes', 0, 1)
            prefix = _SNAPSHOT_MAGIC + len(selected).to_bytes(8, 'little')
            retained = sorted(selected._entries, key=source_indices.__getitem__)
            record = dict(schema='bounded_replay_v2',
                frontier_semantics=frontier_semantics(),
                transitions=[selected._entries[index][0] for index in retained],
                scenes=dict(selected._scenes))
            if active_tasks:
                # Preserve sampler bucket order as well as chronological replay
                # order. Swap-delete task indexes otherwise change across resume.
                ordinals = {source_indices[index]: ordinal
                            for ordinal, index in enumerate(retained)}
                record['task_order'] = {task: [ordinals[index] for index in entries if index in ordinals]
                                        for task, entries in self._task_entries.items()
                                        if any(index in ordinals for index in entries)}
            payload = dumps_transport(record, _prefix=prefix)
            if len(payload) <= max_bytes: return payload
            if not len(selected): raise ValueError('snapshot budget smaller than empty schema')
            selected.evict_oldest()

    @classmethod
    def from_snapshot(cls, payload, max_bytes=8 * 1024**3, *, validate_only=False):
        count = snapshot_sample_count(payload)
        # memoryview removes the envelope without copying the complete bytes.
        record = loads_transport(memoryview(payload)[_SNAPSHOT_HEADER_BYTES:])
        if not isinstance(record, dict) or record.get('schema') != 'bounded_replay_v2':
            raise ValueError('replay schema mismatch')
        require_frontier_semantics(record)
        if not isinstance(record.get('transitions'), list) or len(record['transitions']) != count:
            raise ValueError('replay sample count disagrees with envelope')
        result = cls(max_bytes)
        steps = set()
        active_tasks = set()
        for transition in record['transitions']:
            task = transition_task(transition, record['scenes'])
            if task != 'unclassified':
                active_tasks.add(task)
            if transition.step_index is not None:
                key = (transition.episode_id, transition.step_index)
                if key in steps:
                    raise ValueError('duplicate episode step in replay snapshot')
                steps.add(key)
            result.add(transition, record['scenes'])
            if validate_only:
                # Decode must inspect all contracts, but save/load acceptance needs
                # only one admission ledger at a time. Retain the actual replay only
                # on restore; decoded record objects are released at function exit.
                result.evict_oldest()
        if not validate_only and not active_tasks.issubset(result.task_counts):
            raise ResourceLimitError('replay restore budget lost active task samples',
                                     len(active_tasks.intersection(result.task_counts)), len(active_tasks))
        if 'task_order' in record:
            orders = record['task_order']
            if not isinstance(orders, dict):
                raise ValueError('invalid replay task order')
            seen = set()
            for task, indices in orders.items():
                if not isinstance(indices, list) or not indices:
                    raise ValueError('invalid replay task order bucket')
                for index in indices:
                    if (type(index) is not int or not 0 <= index < count or index in seen or
                            transition_task(record['transitions'][index], record['scenes']) != task):
                        raise ValueError('invalid replay task order index')
                    seen.add(index)
            if len(seen) != count:
                raise ValueError('incomplete replay task order')
            if not validate_only:
                for task, indices in orders.items():
                    kept = [index for index in indices if index in result._entries]
                    if kept:
                        result._task_entries[task] = kept
                        for position, index in enumerate(kept):
                            result._task_bytes += sys.getsizeof(position) - sys.getsizeof(result._task_positions[index])
                            result._task_positions[index] = position
        return None if validate_only else result
