"""Exact enumerated discrete SAC with one optimizer step per effective batch."""
import copy
from .feature_schema import (OBSERVATION_SCHEMA, LEARNER_SCHEMA,
                             frontier_semantics, require_frontier_semantics)
from dataclasses import asdict, dataclass
import math
import numpy as np
import torch
from .batch import pack_observations, pack_privileged
from .config import ModelConfig, LearningConfig, canonical_model_config, canonical_learning_config
from .contracts import Transition
from .model import Actor, Critic, segment_sum


def checked_gradient_norm(parameters, loss, phase):
    """Reduce on device, then make one finite gate before an optimizer step."""
    gradients = [p.grad for p in parameters if p.grad is not None]
    if not gradients:
        raise FloatingPointError(phase + ' has no gradients')
    norms = torch.stack([torch.linalg.vector_norm(gradient.detach()) for gradient in gradients])
    norm = torch.linalg.vector_norm(norms)
    if not bool(torch.isfinite(torch.stack((loss.detach(), norm))).all()):
        raise FloatingPointError(phase + ' nonfinite loss or gradients')
    return norm


def require_finite_parameters(parameters, phase):
    """Check updated tensors before they can become a completed learner version."""
    finite = torch.stack([torch.isfinite(p.detach()).all() for p in parameters])
    if not bool(finite.all()):
        raise FloatingPointError(phase + ' nonfinite parameters after optimizer step')


def soft_values(probs, log_probs, q1, q2, alpha, groups, count, *, entropy_bonus=None):
    if entropy_bonus is None:
        # Preserve the legacy joint path, including its arithmetic order.
        return segment_sum(probs * (torch.minimum(q1, q2) - alpha * log_probs), groups, count)
    return segment_sum(probs * (torch.minimum(q1, q2) + entropy_bonus), groups, count)


def policy_loss(probs, log_probs, q1, q2, alpha, groups, count, *, entropy_bonus=None):
    return -soft_values(probs, log_probs, q1.detach(), q2.detach(),
                        alpha, groups, count, entropy_bonus=entropy_bonus).mean()


@dataclass
class PolicyEntropies:
    """Action-aligned log factors and state-aligned entropies/reference targets."""
    position_log_probs: torch.Tensor
    heading_log_probs: torch.Tensor
    position_entropy: torch.Tensor
    heading_entropy: torch.Tensor
    position_target: torch.Tensor
    heading_target: torch.Tensor


def policy_entropies(policy, batch, position_factor=.1, heading_factor=.1):
    """Decompose a joint policy without probability clamping or actor changes.

    Packed action nodes are global graph indices. Only nodes with valid actions
    contribute. A stable segment logsumexp keeps log p(position) differentiable
    even when its probability underflows. Target factors multiply entropies of
    the uniform *joint-action* reference: w(position)=heading_count/action_count.
    Equal target factors therefore sum to the legacy factor * log(action_count),
    including ragged legal heading sets, independently of the current policy.
    """
    logp, probs = policy.log_probs, policy.probs
    nodes, groups, count = batch.action_nodes, batch.action_groups, batch.size
    node_count = len(batch.graph.features)
    heading_counts = segment_sum(torch.ones_like(probs), nodes, node_count)
    maxima = logp.new_full((node_count,), -torch.inf).scatter_reduce_(
        0, nodes, logp.detach(), reduce='amax', include_self=True)
    maxima = torch.where(heading_counts > 0, maxima, 0.)
    total = segment_sum((logp - maxima[nodes]).exp(), nodes, node_count)
    # Empty graph nodes never enter an action, but must still have finite log
    # values/backward derivatives. Nonempty totals are >= 1 after centering.
    total = torch.where(heading_counts > 0, total, 1.)
    position_logp = (maxima + total.log())[nodes]
    action_counts = segment_sum(torch.ones_like(probs), groups, count)
    # With one legal position p(position)=1 exactly. Avoid roundoff-generated
    # entropy and temperature updates for a component that has no choice.
    position_logp = torch.where(heading_counts[nodes] == action_counts[groups],
                               torch.zeros_like(position_logp), position_logp)
    heading_logp = logp - position_logp
    position_entropy = -segment_sum(probs * position_logp, groups, count)
    heading_entropy = -segment_sum(probs * heading_logp, groups, count)
    log_heading_counts = heading_counts[nodes].log()
    log_action_counts = action_counts[groups].log()
    position_target = position_factor * segment_sum(
        (log_action_counts - log_heading_counts) / action_counts[groups], groups, count)
    heading_target = heading_factor * segment_sum(
        log_heading_counts / action_counts[groups], groups, count)
    return PolicyEntropies(position_logp, heading_logp, position_entropy, heading_entropy,
                           position_target, heading_target)


def factorized_entropy_bonus(entropies, alpha_position, alpha_heading):
    """Per-action entropy bonus shared by actor and all bootstrap targets."""
    return (-alpha_position * entropies.position_log_probs -
            alpha_heading * entropies.heading_log_probs)


def temperature_loss(log_alpha, entropy, target_entropy):
    # Gradient descent reduces alpha when entropy exceeds its target.
    return (log_alpha * (entropy - target_entropy).detach()).mean()


@torch.no_grad()
def retrace_coefficients(log_pi, log_mu):
    """Lambda=1 importance correction for the actual sampled joint action."""
    return (log_pi - log_mu).clamp(max=0).exp()


@torch.no_grad()
def soft_retrace_target(rewards, discounts, next_values, next_q, coefficients):
    """Detached finite soft Retrace for one sequence of at most eight steps.

    ``next_values[i]`` is E_pi[min(target Q) - alpha log pi] at s_(i+1).
    The last two arrays contain only the m-1 sampled successor actions:
    raw min(target Q) and min(1, pi / mu), with no extra entropy subtraction.
    ``discounts`` already masks true termination, but not time truncation.
    """
    length = rewards.numel()
    if (not 1 <= length <= 8 or any(t.ndim != 1 for t in
            (rewards, discounts, next_values, next_q, coefficients)) or
            discounts.numel() != length or next_values.numel() != length or
            next_q.numel() != length - 1 or coefficients.numel() != length - 1):
        raise ValueError('invalid soft Retrace sequence tensor shapes')
    result = rewards[-1] + discounts[-1] * next_values[-1]
    for i in range(length - 2, -1, -1):
        result = rewards[i] + discounts[i] * (
            next_values[i] + coefficients[i] * (result - next_q[i]))
    return result


@torch.no_grad()
def policy_diagnostics(policy, batch, bound, entropies=None):
    """State-averaged diagnostics; raw gradients include effective-batch scaling.

    Saturation means |tanh(centered_score / C)| >= .95 (zero for C=0).
    Position probabilities sum valid headings at each packed graph node.
    Only the final update metrics transfer synchronizes with the host.
    """
    probs, raw = policy.probs, policy.raw_logits
    groups, size = batch.action_groups, batch.size
    counts = raw.new_tensor(batch.action_counts).clamp_min(1)
    position_probs = segment_sum(probs, batch.action_nodes, len(batch.graph.features))
    position_entropy = -segment_sum(position_probs * position_probs.clamp_min(1e-30).log(),
                                    batch.graph.groups, size)
    joint_entropy = -segment_sum(probs * policy.log_probs, groups, size)
    position_max = raw.new_zeros(size).scatter_reduce_(
        0, batch.graph.groups, position_probs, reduce='amax', include_self=True)
    maxima = raw.new_full((size,), -torch.inf).scatter_reduce_(
        0, groups, raw, reduce='amax', include_self=True)
    minima = raw.new_full((size,), torch.inf).scatter_reduce_(
        0, groups, raw, reduce='amin', include_self=True)
    spans = torch.where(torch.isfinite(maxima), maxima - minima, 0.)
    saturated = (policy.logits.abs() >= .95 * bound).to(raw.dtype) if bound else torch.zeros_like(raw)
    saturation = segment_sum(saturated, groups, size) / counts
    gradient = raw.grad.abs()
    gradient_mean = (segment_sum(gradient, groups, size) / counts).mean()
    gradient_max = gradient.amax() if gradient.numel() else raw.new_zeros(())
    heading_entropy = joint_entropy-position_entropy
    if entropies is not None:
        position_entropy, heading_entropy = entropies.position_entropy, entropies.heading_entropy
    return torch.stack((position_entropy.mean(), heading_entropy.mean(),
        position_max.mean(), spans.mean(), saturation.mean(), gradient_mean, gradient_max))


class SACLearner:
    """Learner owns only networks/optimizers. Scheduler, replay and RNG are external.

    ``actor_state()`` returns an owned CPU snapshot and completed-update version.
    The scheduler chooses the 16-update publication interval. ``state_dict()``
    returns an owned, Torch-serializable full learner snapshot. Restore uses the
    current microbatch/rate/Polyak controls; initial alpha is new-training-only.
    Model and experience-objective settings must remain compatible.
    """
    def __init__(self, model_config=ModelConfig(), learning_config=LearningConfig(), *, device='cpu'):
        self.model_config, self.learning_config = model_config, learning_config
        self.device = torch.device(device)
        self.actor = Actor(model_config).to(self.device)
        self.q1, self.q2 = (Critic(model_config).to(self.device) for _ in range(2))
        self.target1, self.target2 = copy.deepcopy(self.q1), copy.deepcopy(self.q2)
        self.target1.requires_grad_(False)
        self.target2.requires_grad_(False)
        self.log_alpha = torch.tensor(math.log(learning_config.initial_alpha), device=self.device,
                                      requires_grad=True)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=learning_config.learning_rate)
        self.q1_optimizer = torch.optim.Adam(self.q1.parameters(), lr=learning_config.learning_rate)
        self.q2_optimizer = torch.optim.Adam(self.q2.parameters(), lr=learning_config.learning_rate)
        self.alpha_optimizer = torch.optim.Adam([self.log_alpha], lr=self.temperature_learning_rate)
        if learning_config.entropy_mode == 'position_heading':
            self.log_alpha_heading = self.log_alpha.detach().clone().requires_grad_()
            self.heading_alpha_optimizer = torch.optim.Adam(
                [self.log_alpha_heading], lr=self.temperature_learning_rate)
        self.updates = 0

    @property
    def alpha(self):
        return self.log_alpha.detach().exp()

    @property
    def alpha_position(self):
        return self.alpha

    @property
    def alpha_heading(self):
        if self.learning_config.entropy_mode == 'joint':
            return self.alpha
        return self.log_alpha_heading.detach().exp()

    @property
    def temperature_learning_rate(self):
        configured = self.learning_config.temperature_learning_rate
        return self.learning_config.learning_rate if configured is None else configured

    def entropy_bonus(self, policy, batch):
        """Public per-action bonus for evaluation; joint keeps its original form."""
        if self.learning_config.entropy_mode == 'joint':
            return -self.alpha * policy.log_probs
        return factorized_entropy_bonus(policy_entropies(policy, batch),
                                        self.alpha_position, self.alpha_heading)

    def _split_entropy_bonus(self, policy, batch):
        return self.entropy_bonus(policy, batch) if self.learning_config.entropy_mode == 'position_heading' else None

    @torch.no_grad()
    def td_targets(self, transitions, scenes):
        targets = torch.tensor([t.reward for t in transitions], device=self.device)
        active = [i for i, t in enumerate(transitions) if not t.terminated]
        if active:
            observations = [transitions[i].next_observation for i in active]
            if any(len(o.action_nodes) == 0 for o in observations):
                raise ValueError("nonterminal successor requires valid actions")
            states = [transitions[i].next_privileged for i in active]
            batch = pack_observations(observations, self.device)
            truth = pack_privileged(observations, states, scenes, self.device)
            policy = self.actor.forward_packed(batch)
            values = soft_values(policy.probs, policy.log_probs,
                self.target1.forward_packed(batch, truth), self.target2.forward_packed(batch, truth),
                self.alpha, batch.action_groups, batch.size,
                entropy_bonus=self._split_entropy_bonus(policy, batch))
            targets[torch.tensor(active, device=self.device)] += self.learning_config.gamma * values
        return targets

    def _sequences(self, samples):
        """Validate before optimizer changes; replay owns full state adjacency."""
        sequences = []
        for sample in samples:
            sequence = (sample,) if isinstance(sample, Transition) else sample
            if (not isinstance(sequence, tuple) or
                    not 1 <= len(sequence) <= self.learning_config.return_steps or
                    any(not isinstance(t, Transition) for t in sequence)):
                raise ValueError('invalid replay sequence length or entries')
            if any(not 0 <= t.action < len(t.observation.action_nodes) for t in sequence):
                raise ValueError('replay action must index the frozen valid action list')
            if len(sequence) > 1:
                if any(t.step_index is None or t.behavior_log_prob is None for t in sequence):
                    raise ValueError('multi-step sequence requires behavior metadata')
                for previous, following in zip(sequence, sequence[1:]):
                    if (previous.terminated or previous.truncated or
                            previous.episode_id != following.episode_id or
                            previous.step_index + 1 != following.step_index or
                            previous.privileged.scene_id != following.privileged.scene_id):
                        raise ValueError('replay sequence crosses an episode or step boundary')
            sequences.append(sequence)
        return sequences

    @torch.no_grad()
    def retrace_targets(self, sequences, scenes):
        """Return one target and coefficient mass per anchor, without graph retention.

        Only graph *references* are flattened; every target forward contains at
        most microbatch_size states. Actor, alpha and target Q stay unchanged
        throughout the critic phase. Legacy singleton sequences use exact TD.
        """
        sequences = self._sequences(sequences)
        if not sequences:
            raise ValueError('soft Retrace requires a nonempty sequence batch')
        size = self.learning_config.microbatch_size
        if all(len(sequence) == 1 for sequence in sequences):
            anchors = [sequence[0] for sequence in sequences]
            targets = torch.cat([self.td_targets(anchors[i:i + size], scenes)
                                 for i in range(0, len(anchors), size)])
            return targets, torch.ones_like(targets)

        transitions, requests, ranges = [], [], []
        for sequence in sequences:
            start = len(transitions)
            transitions.extend(sequence)
            ranges.append((start, len(transitions)))
            for i, transition in enumerate(sequence):
                if transition.terminated:
                    continue
                if len(transition.next_observation.action_nodes) == 0:
                    raise ValueError('nonterminal successor requires valid actions')
                following = sequence[i + 1] if i + 1 < len(sequence) else None
                requests.append((start + i, transition.next_observation,
                    transition.next_privileged, following))
        rewards = torch.tensor([t.reward for t in transitions], device=self.device)
        discounts = rewards.new_tensor([self.learning_config.gamma * (not t.terminated)
                                        for t in transitions])
        values, selected, coefficients = (torch.zeros_like(rewards) for _ in range(3))
        for offset in range(0, len(requests), size):
            chunk = requests[offset:offset + size]
            observations = [request[1] for request in chunk]
            batch = pack_observations(observations, self.device)
            truth = pack_privileged(observations, [request[2] for request in chunk], scenes, self.device)
            policy = self.actor.forward_packed(batch)
            minimum = torch.minimum(self.target1.forward_packed(batch, truth),
                                    self.target2.forward_packed(batch, truth))
            soft = soft_values(policy.probs, policy.log_probs, minimum, minimum, self.alpha,
                batch.action_groups, batch.size, entropy_bonus=self._split_entropy_bonus(policy, batch))
            destinations = torch.tensor([request[0] for request in chunk], device=self.device)
            values[destinations] = soft
            # No action is sampled at the final bootstrap state. Its dummy
            # gather is discarded; only interior states enter the correction.
            indices = torch.tensor(np.cumsum((0,) + batch.action_counts[:-1]) +
                [request[3].action if request[3] is not None else 0 for request in chunk],
                device=self.device)
            log_mu = policy.log_probs.new_tensor([
                request[3].behavior_log_prob if request[3] is not None else 0. for request in chunk])
            selected[destinations] = minimum[indices]
            coefficients[destinations] = retrace_coefficients(policy.log_probs[indices], log_mu)
        targets, masses = [], []
        for start, stop in ranges:
            correction = coefficients[start:stop - 1]
            targets.append(soft_retrace_target(rewards[start:stop], discounts[start:stop],
                values[start:stop], selected[start:stop - 1], correction))
            masses.append(1 + correction.cumprod(0).sum())
        return torch.stack(targets), torch.stack(masses)

    def update(self, transitions, scenes):
        config = self.learning_config
        if len(transitions) != config.batch_size:
            raise ValueError("a complete update requires 64 transitions")
        sequences = self._sequences(transitions)
        if any(obs.schema != self.model_config.observation_schema
               for sequence in sequences for transition in sequence
               for obs in (transition.observation, transition.next_observation)):
            raise ValueError('SAC observation schema does not match model architecture')
        transitions = [sequence[0] for sequence in sequences]
        chunks = [transitions[i:i+config.microbatch_size]
                  for i in range(0,len(transitions),config.microbatch_size)]
        # Scalar diagnostics stay on-device until the single final transfer.
        metrics = torch.zeros(13, device=self.device)
        coefficient_mass = torch.zeros((), device=self.device)
        moments = torch.zeros((5, 2), device=self.device)
        samples_for_diagnostics = [[] for _ in range(3)]
        self.q1_optimizer.zero_grad(set_to_none=True)
        self.q2_optimizer.zero_grad(set_to_none=True)
        for offset, chunk in zip(range(0, len(transitions), config.microbatch_size), chunks):
            weight = len(chunk) / len(transitions)
            if config.return_steps == 1:
                targets = self.td_targets(chunk, scenes)
                coefficient_mass += len(chunk)
            else:
                targets, masses = self.retrace_targets(sequences[offset:offset + len(chunk)], scenes)
                coefficient_mass += masses.sum()
            moments[2] += torch.stack((targets.sum(), targets.square().sum()))
            samples_for_diagnostics[2].append(targets.detach())
            batch = pack_observations([t.observation for t in chunk], self.device)
            truth = pack_privileged([t.observation for t in chunk], [t.privileged for t in chunk], scenes, self.device)
            indices = torch.tensor(np.cumsum((0,) + batch.action_counts[:-1]) +
                [t.action for t in chunk], device=self.device)
            # Independent networks permit immediate backward/release, without
            # retaining both critics' activations or changing optimizer cadence.
            for i, critic in enumerate((self.q1, self.q2)):
                selected = critic.forward_packed(batch, truth)[indices]
                loss = (selected - targets).square().mean()
                (loss * weight).backward()
                metrics[i] += loss.detach() * weight
                detached = selected.detach()
                samples_for_diagnostics[i].append(detached)
                td_error = detached - targets
                moments[i] += torch.stack((detached.sum(), detached.square().sum()))
                moments[i + 3] += torch.stack((td_error.sum(), td_error.square().sum()))
                del selected, loss, detached, td_error
            metrics[5] += targets.mean() * weight
        q1_norm = checked_gradient_norm(self.q1.parameters(), metrics[0], 'q1')
        q2_norm = checked_gradient_norm(self.q2.parameters(), metrics[1], 'q2')
        self.q1_optimizer.step()
        require_finite_parameters(self.q1.parameters(), 'q1')
        self.q2_optimizer.step()
        require_finite_parameters(self.q2.parameters(), 'q2')
        # Clear critic grads before the actor phase: detached Q cannot pollute them.
        self.q1_optimizer.zero_grad(set_to_none=True)
        self.q2_optimizer.zero_grad(set_to_none=True)
        self.actor_optimizer.zero_grad(set_to_none=True)
        self.alpha_optimizer.zero_grad(set_to_none=True)
        split_entropy = config.entropy_mode == 'position_heading'
        if split_entropy:
            self.heading_alpha_optimizer.zero_grad(set_to_none=True)
        entropy_gap = torch.zeros((), device=self.device)
        heading_entropy_gap = torch.zeros((), device=self.device)
        entropy_targets = torch.zeros(2, device=self.device)
        for chunk in chunks:
            weight = len(chunk) / len(transitions)
            batch = pack_observations([t.observation for t in chunk], self.device)
            truth = pack_privileged([t.observation for t in chunk], [t.privileged for t in chunk], scenes, self.device)
            with torch.no_grad():
                q1, q2 = self.q1.forward_packed(batch, truth), self.q2.forward_packed(batch, truth)
            policy = self.actor.forward_packed(batch)
            policy.raw_logits.retain_grad()
            entropies = policy_entropies(policy, batch,
                config.position_target_entropy_factor if split_entropy else config.target_entropy_factor,
                config.heading_target_entropy_factor if split_entropy else config.target_entropy_factor)
            bonus = factorized_entropy_bonus(entropies, self.alpha_position, self.alpha_heading) if split_entropy else None
            loss = policy_loss(policy.probs, policy.log_probs, q1, q2, self.alpha,
                               batch.action_groups, batch.size, entropy_bonus=bonus)
            (loss * weight).backward()
            entropy = -segment_sum(policy.probs.detach() * policy.log_probs.detach(), batch.action_groups, batch.size)
            target_entropy = torch.tensor(np.log(batch.action_counts) * config.target_entropy_factor,
                                          dtype=entropy.dtype, device=self.device)
            if split_entropy:
                entropy_gap += (entropies.position_entropy.detach()-entropies.position_target).mean() * weight
                heading_entropy_gap += (entropies.heading_entropy.detach()-entropies.heading_target).mean() * weight
            else:
                entropy_gap += (entropy-target_entropy).mean() * weight
            entropy_targets += torch.stack((entropies.position_target.mean(),
                entropies.heading_target.mean())) * weight
            metrics[2] += loss.detach() * weight
            metrics[4] += entropy.mean() * weight
            diagnostics = policy_diagnostics(policy, batch, self.model_config.actor_score_bound,
                                             entropies if split_entropy else None)
            metrics[6:12] += diagnostics[:6] * weight
            metrics[12] = torch.maximum(metrics[12], diagnostics[6])
        actor_norm = checked_gradient_norm(self.actor.parameters(), metrics[2], 'actor')
        self.actor_optimizer.step()
        require_finite_parameters(self.actor.parameters(), 'actor')
        alpha_loss = temperature_loss(self.log_alpha, entropy_gap, 0.0)
        if split_entropy:
            alpha_loss = alpha_loss + temperature_loss(self.log_alpha_heading, heading_entropy_gap, 0.0)
        alpha_loss.backward()
        temperatures = [self.log_alpha] + ([self.log_alpha_heading] if split_entropy else [])
        temperature_norm = checked_gradient_norm(temperatures, alpha_loss, 'temperature')
        self.alpha_optimizer.step()
        require_finite_parameters([self.log_alpha], 'temperature')
        if split_entropy:
            self.heading_alpha_optimizer.step()
            require_finite_parameters([self.log_alpha_heading], 'temperature')
        metrics[3] = alpha_loss.detach()
        with torch.no_grad():
            self.log_alpha.clamp_(max=math.log(config.effective_maximum_position_alpha))
            if split_entropy:
                self.log_alpha_heading.clamp_(max=math.log(config.maximum_alpha))
            for target, source in ((self.target1,self.q1),(self.target2,self.q2)):
                for target_p, source_p in zip(target.parameters(), source.parameters()):
                    target_p.lerp_(source_p, config.polyak)
        require_finite_parameters((*self.target1.parameters(), *self.target2.parameters()), 'target')
        standard_deviations = (moments[:, 1] / len(transitions) -
                              (moments[:, 0] / len(transitions)).square()).clamp_min(0).sqrt()
        q1_samples, q2_samples, target_samples = (torch.cat(chunks) for chunks in samples_for_diagnostics)
        absolute_td = torch.cat(((q1_samples - target_samples).abs(),
                                 (q2_samples - target_samples).abs()))
        percentiles = metrics.new_tensor([.5, .95, .99])
        quantiles = torch.cat([torch.quantile(values, percentiles)
            for values in (q1_samples, q2_samples, target_samples, absolute_td)])
        diagnostics = torch.cat((torch.stack((actor_norm, q1_norm, q2_norm, temperature_norm)),
            standard_deviations,
            torch.stack(((self.alpha_position == 0).to(metrics.dtype),
                         (self.alpha_heading == 0).to(metrics.dtype),
                         self.log_alpha.detach(),
                         (self.log_alpha_heading if split_entropy else self.log_alpha).detach())), quantiles))
        if not bool(torch.isfinite(torch.cat((metrics, diagnostics))).all()):
            raise FloatingPointError('nonfinite update diagnostics')
        self.updates += 1
        values = torch.cat((metrics, self.alpha.reshape(1), coefficient_mass.reshape(1),
            self.alpha_position.reshape(1), self.alpha_heading.reshape(1), entropy_targets, diagnostics)).detach().cpu().tolist()
        return dict(zip(('critic1_loss','critic2_loss','actor_loss','alpha_loss','entropy','target_mean',
                        'position_entropy','heading_conditional_entropy','position_max_probability',
                        'raw_score_span','score_saturation_fraction','raw_score_gradient_abs_mean',
                        'raw_score_gradient_abs_max','alpha','trace_coefficient_mass_sum',
                        'alpha_position','alpha_heading','position_entropy_target','heading_entropy_target',
                        'actor_grad_norm','q1_grad_norm','q2_grad_norm','temperature_grad_norm',
                        'q1_selected_std','q2_selected_std','td_target_std','q1_td_error_std','q2_td_error_std',
                        'alpha_position_underflow','alpha_heading_underflow','log_alpha_position','log_alpha_heading',
                        'q1_p50','q1_p95','q1_p99','q2_p50','q2_p95','q2_p99',
                        'td_target_p50','td_target_p95','td_target_p99','td_p50','td_p95','td_p99'), values),
                    updates=self.updates, numeric_guard_version=1, numeric_checked_updates=self.updates,
                    batch_size=len(transitions), trace_samples=len(sequences),
                    trace_length_sum=sum(map(len, sequences)),
                    trace_multi_count=sum(len(sequence) > 1 for sequence in sequences),
                    trace_legacy_count=sum(sequence[0].behavior_log_prob is None for sequence in sequences))

    def actor_state(self):
        return {'schema':self.model_config.observation_schema, 'frontier_semantics':frontier_semantics(), 'model_config':asdict(self.model_config),
                'version':self.updates,
                'state_dict':{k:v.detach().cpu().clone() for k,v in self.actor.state_dict().items()}}

    def state_dict(self):
        result = {'schema':LEARNER_SCHEMA, 'frontier_semantics':frontier_semantics(), 'model_config':asdict(self.model_config),
                  'learning_config':asdict(self.learning_config), 'updates':self.updates,
                  'log_alpha':self.log_alpha.detach().clone()}
        for name in ('actor','q1','q2','target1','target2','actor_optimizer',
                     'q1_optimizer','q2_optimizer','alpha_optimizer'):
            result[name] = getattr(self,name).state_dict()
        if self.learning_config.entropy_mode == 'position_heading':
            result['log_alpha_heading'] = self.log_alpha_heading.detach().clone()
            result['heading_alpha_optimizer'] = self.heading_alpha_optimizer.state_dict()
        return copy.deepcopy(result)

    def load_state_dict(self, state):
        require_frontier_semantics(state)
        if (state.get('schema') != LEARNER_SCHEMA or
                canonical_model_config(state['model_config']) != asdict(self.model_config)):
            raise ValueError("incompatible learner model schema")
        try:
            saved = canonical_learning_config(state['learning_config'])
        except ValueError as error:
            raise ValueError('incompatible SAC semantics') from error
        current = canonical_learning_config(asdict(self.learning_config))
        for name in ('microbatch_size', 'learning_rate', 'polyak', 'initial_alpha', 'temperature_learning_rate'):
            saved.pop(name)
            current.pop(name)
        if saved != current:
            raise ValueError("incompatible SAC semantics")
        split_entropy = self.learning_config.entropy_mode == 'position_heading'
        if split_entropy and any(key not in state for key in ('log_alpha_heading','heading_alpha_optimizer')):
            raise ValueError('split entropy checkpoint requires heading temperature and optimizer state')
        # Optimizer load can retain references to input tensors; own the checkpoint.
        state = copy.deepcopy(state)
        for name in ('actor','q1','q2','target1','target2','actor_optimizer',
                     'q1_optimizer','q2_optimizer','alpha_optimizer'):
            getattr(self,name).load_state_dict(state[name])
        if split_entropy:
            self.heading_alpha_optimizer.load_state_dict(state['heading_alpha_optimizer'])
        # Loading Adam restores parameter-group metadata as well as history.
        # Preserve saved moments/steps, but apply the caller's continuation rate.
        for optimizer in (self.actor_optimizer, self.q1_optimizer, self.q2_optimizer):
            for group in optimizer.param_groups:
                group['lr'] = self.learning_config.learning_rate
        temperature_optimizers = (self.alpha_optimizer, self.heading_alpha_optimizer) if split_entropy else (self.alpha_optimizer,)
        for optimizer in temperature_optimizers:
            for group in optimizer.param_groups:
                group['lr'] = self.temperature_learning_rate
        with torch.no_grad():
            self.log_alpha.copy_(state['log_alpha'])
            if split_entropy:
                self.log_alpha_heading.copy_(state['log_alpha_heading'])
        self.updates = int(state['updates'])
