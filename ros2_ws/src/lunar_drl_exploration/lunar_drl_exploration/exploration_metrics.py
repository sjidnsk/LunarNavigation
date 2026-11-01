"""Read-only exploration curves; no actions, rewards, or stopping decisions."""
import math
import time


class ExplorationTrace:
    def __init__(self, initial_coverage, budget=512):
        if not 0 <= initial_coverage <= 1 or type(budget) is not int or budget <= 0:
            raise ValueError('finite coverage and positive decision budget required')
        self.samples = [[0., 0., float(initial_coverage)]]
        self.decisions = [[0, float(initial_coverage)]]
        self.budget = budget
        self.inference_ms = []
        self.collection_wall_s = 0.

    def sample(self, distance_m, simulation_s, coverage):
        values = [float(distance_m), float(simulation_s), float(coverage)]
        old = self.samples[-1]
        if (not all(math.isfinite(v) for v in values) or not 0 <= values[2] <= 1
                or values[0] < old[0]-1e-9 or values[1] < old[1]-1e-9
                or values[2] < old[2]-1e-12):
            raise ValueError('nonfinite or nonmonotonic exploration trace')
        values = [max(v, p) for v, p in zip(values, old)]
        if values != old:
            self.samples.append(values)

    def decision(self, number, coverage):
        if type(number) is not int or not self.decisions[-1][0] < number <= self.budget:
            raise ValueError('decision index outside fixed budget')
        if not math.isfinite(coverage) or not self.decisions[-1][1] <= coverage <= 1:
            raise ValueError('invalid decision coverage')
        self.decisions.append([number, float(coverage)])

    def summary(self, reference_area_m2):
        thresholds = {}
        for percent in (90, 95, 99):
            found = next((v for v in self.samples if v[2] >= percent/100.), None)
            thresholds[str(percent)] = (None if found is None else
                dict(distance_m=found[0], simulation_s=found[1]))
        tail = None
        if thresholds['95'] is not None and thresholds['99'] is not None:
            tail = {k:thresholds['99'][k]-thresholds['95'][k]
                    for k in ('distance_m', 'simulation_s')}
        anchor = self.samples[0]
        no_gain_d = no_gain_t = 0.
        for point in self.samples[1:]:
            if point[2] > anchor[2]:
                anchor = point
            else:
                no_gain_d = max(no_gain_d, point[0]-anchor[0])
                no_gain_t = max(no_gain_t, point[1]-anchor[1])
        curve = self.decisions+[[self.budget, self.decisions[-1][1]]]
        auc = sum((b[0]-a[0])*a[1] for a,b in zip(curve, curve[1:]))/self.budget
        end = self.samples[-1]
        return dict(schema='exploration_metrics_v1',
            sampling='first confirmed native-map update; no interpolation',
            distance_time_origin='policy start, excluding initialization',
            sample_count=len(self.samples), thresholds=thresholds,
            tail_95_to_99=tail, policy_distance_m=end[0], policy_simulation_s=end[1],
            final_observed_coverage=end[2],
            new_area_per_meter_m2=((end[2]-self.samples[0][2])*reference_area_m2/end[0]
                                   if end[0] > 0 else None),
            max_no_gain_distance_m=no_gain_d, max_no_gain_simulation_s=no_gain_t,
            decision_auc_512=auc, decision_budget=self.budget,
            terminal_auc_rule='hold last observed coverage; failures remain in denominator',
            inference_ms=self.inference_ms, collection_wall_s=self.collection_wall_s)


def attach_trace(env, initial_coverage):
    """Observe already-consumed native coverage without modifying its input."""
    import numpy as np
    trace = ExplorationTrace(initial_coverage, env.episode_budget)
    distance0, time0 = env.plant.distance_m, env.plant.simulation_s
    original_apply = env._apply_known
    previous = None

    def sample(coverage=None):
        started = time.perf_counter()
        if coverage is None:
            identity = env.core.coverage.identity
            resolution, origin = identity[1:]
            offset = (np.asarray(env.terrain.origin[:2])-origin[:2])/resolution
            rounded = np.rint(offset).astype(int)
            if np.any(np.abs(offset-rounded) > 1e-6):
                raise ValueError('metric lattice differs from reference')
            x, y = map(int, rounded)
            h, w = env.terrain.shape
            mask = env.core.coverage.mask((x, y, x+w, y+h))
            coverage = env.reference.coverage_ratio(env.reference.pack(mask))
        trace.sample(env.plant.distance_m-distance0, env.plant.simulation_s-time0, coverage)
        trace.collection_wall_s += time.perf_counter()-started

    def applied(snapshot):
        nonlocal previous
        result = original_apply(snapshot)
        key = (env.core.coverage.revision, env.core.coverage.identity)
        if key != previous:
            sample()
            previous = key
        return result

    env._apply_known = applied
    return trace, sample


def curve_auc(samples, coordinate, budget):
    """Left-continuous coverage integral at a fixed common physical budget."""
    if not samples or coordinate not in (0, 1) or not math.isfinite(budget) or budget <= 0:
        raise ValueError('nonempty curve and positive common budget required')
    total = 0.
    for a, b in zip(samples, samples[1:]):
        total += max(0., min(b[coordinate], budget)-min(a[coordinate], budget))*a[2]
    total += max(0., budget-samples[-1][coordinate])*samples[-1][2]
    return total/budget
