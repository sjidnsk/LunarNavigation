# Frozen numerical oracle from9c80dd8c; execute with sac module globals.
def legacy_update(self, transitions, scenes):
    config = self.learning_config
    if len(transitions) != config.batch_size:
        raise ValueError('a complete update requires 64 transitions')
    sequences = self._sequences(transitions)
    transitions = [sequence[0] for sequence in sequences]
    chunks = [transitions[i:i + config.microbatch_size] for i in range(0, len(transitions), config.microbatch_size)]
    metrics = torch.zeros(13, device=self.device)
    coefficient_mass = torch.zeros((), device=self.device)
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
        batch = pack_observations([t.observation for t in chunk], self.device)
        truth = pack_privileged([t.observation for t in chunk], [t.privileged for t in chunk], scenes, self.device)
        indices = torch.tensor(np.cumsum((0,) + batch.action_counts[:-1]) + [t.action for t in chunk], device=self.device)
        for i, critic in enumerate((self.q1, self.q2)):
            selected = critic.forward_packed(batch, truth)[indices]
            loss = (selected - targets).square().mean()
            (loss * weight).backward()
            metrics[i] += loss.detach() * weight
            del selected, loss
        metrics[5] += targets.mean() * weight
    self.q1_optimizer.step()
    self.q2_optimizer.step()
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
            q1, q2 = (self.q1.forward_packed(batch, truth), self.q2.forward_packed(batch, truth))
        policy = self.actor.forward_packed(batch)
        policy.raw_logits.retain_grad()
        entropies = policy_entropies(policy, batch, config.position_target_entropy_factor if split_entropy else config.target_entropy_factor, config.heading_target_entropy_factor if split_entropy else config.target_entropy_factor)
        bonus = factorized_entropy_bonus(entropies, self.alpha_position, self.alpha_heading) if split_entropy else None
        loss = policy_loss(policy.probs, policy.log_probs, q1, q2, self.alpha, batch.action_groups, batch.size, entropy_bonus=bonus)
        (loss * weight).backward()
        entropy = -segment_sum(policy.probs.detach() * policy.log_probs.detach(), batch.action_groups, batch.size)
        target_entropy = torch.tensor(np.log(batch.action_counts) * config.target_entropy_factor, dtype=entropy.dtype, device=self.device)
        if split_entropy:
            entropy_gap += (entropies.position_entropy.detach() - entropies.position_target).mean() * weight
            heading_entropy_gap += (entropies.heading_entropy.detach() - entropies.heading_target).mean() * weight
        else:
            entropy_gap += (entropy - target_entropy).mean() * weight
        entropy_targets += torch.stack((entropies.position_target.mean(), entropies.heading_target.mean())) * weight
        metrics[2] += loss.detach() * weight
        metrics[4] += entropy.mean() * weight
        diagnostics = policy_diagnostics(policy, batch, self.model_config.actor_score_bound, entropies if split_entropy else None)
        metrics[6:12] += diagnostics[:6] * weight
        metrics[12] = torch.maximum(metrics[12], diagnostics[6])
    self.actor_optimizer.step()
    alpha_loss = temperature_loss(self.log_alpha, entropy_gap, 0.0)
    if split_entropy:
        alpha_loss = alpha_loss + temperature_loss(self.log_alpha_heading, heading_entropy_gap, 0.0)
    alpha_loss.backward()
    self.alpha_optimizer.step()
    if split_entropy:
        self.heading_alpha_optimizer.step()
    metrics[3] = alpha_loss.detach()
    with torch.no_grad():
        self.log_alpha.clamp_(max=math.log(config.maximum_alpha))
        if split_entropy:
            self.log_alpha_heading.clamp_(max=math.log(config.maximum_alpha))
        for target, source in ((self.target1, self.q1), (self.target2, self.q2)):
            for target_p, source_p in zip(target.parameters(), source.parameters()):
                target_p.lerp_(source_p, config.polyak)
    self.updates += 1
    values = torch.cat((metrics, self.alpha.reshape(1), coefficient_mass.reshape(1), self.alpha_position.reshape(1), self.alpha_heading.reshape(1), entropy_targets)).detach().cpu().tolist()
    return dict(zip(('critic1_loss', 'critic2_loss', 'actor_loss', 'alpha_loss', 'entropy', 'target_mean', 'position_entropy', 'heading_conditional_entropy', 'position_max_probability', 'raw_score_span', 'score_saturation_fraction', 'raw_score_gradient_abs_mean', 'raw_score_gradient_abs_max', 'alpha', 'trace_coefficient_mass_sum', 'alpha_position', 'alpha_heading', 'position_entropy_target', 'heading_entropy_target'), values), updates=self.updates, batch_size=len(transitions), trace_samples=len(sequences), trace_length_sum=sum(map(len, sequences)), trace_multi_count=sum((len(sequence) > 1 for sequence in sequences)), trace_legacy_count=sum((sequence[0].behavior_log_prob is None for sequence in sequences)))
