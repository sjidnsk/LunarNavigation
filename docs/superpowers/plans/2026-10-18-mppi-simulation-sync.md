# MPPI dynamic simulation pacing

User correction: target_rtf=30 is the existing dynamic ceiling, not a fixed actual speed. Keep it. Do not adopt the discarded fixed1/3/5 proposals or deadline division by target RTF. The independent200 remains authorized, not yet launched.

Root cause: RosExplorationEnv._pace includes environment-thread work but not the asynchronous MPPI solver. Physics can advance while the control process is busy. Preserve 0.05s physical integration/sensor samples, original sampler/weights/model and40ms wall solve budget. Add an opt-in simulation-only step completion barrier.

Controller: default-off parameter requires MPPI/use_sim_time. Timer publishes actual Twist as before; only after a successful publishing tick emits a same-stamp acknowledgment with the exact actual command. No ACK on exception/no publication. Separate reliable transient-local ACK topic and a retained initial zero-command handshake cover discovery and timer-created-after-first-clock races.

Environment: default-off constructor option; require integration_step=.05 to match20Hz. Publish physical frame once; wait for exact current-stamp finite ACK using Condition, install its command for next integration, ignore untagged Twist in sync mode. While waiting, resend only identical clock/odom/TF for discovery; never reintegrate/resample. Check owned-process/spin/cancel errors and wall timeout. Existing pacing includes the wait, so actual RTF falls naturally below ceiling30.

Verification: red/green controller and environment tests; startup without advancing clock; wrong/future stamp, invalid commands, delayed Twist, timeout/cancel, step-period constraint. Full controller and relevant interface tests, isolated Humble build. Native moon/cave single-action dynamic30 probes followed by8 native workers; inspect actual speed and control period. Freeze exact runtime and actor/test manifest before starting persistent8-worker200, error_policy=count_failure. No production default cutover or merge/push.

Boundary: synchronization deliberately changes old asynchronous timing behavior; do not claim old ROS trajectories or deadline counts remain identical. Physical step/sensor cadence and optimizer/kernel parameters remain unchanged.
