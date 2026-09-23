"""ROS-free acceptance predicates for recorded execution evidence."""
import math


def plan_found_with_active_reference(session, references, feedback, diagnostics):
    revisions = {revision for (reference_session, revision) in references
                 if reference_session == session}
    # Feedback belongs to this Action callback; diagnostics are a shared topic.
    return bool(revisions) and (
        any(f['reason_code'] == 'PLAN_FOUND' and f['active_segment_revision'] in revisions
            for f in feedback) or
        any(d.get('cycle_result') == 'PLAN_FOUND' and d.get('session_id') == session and
            d.get('segment_revision') in {str(revision) for revision in revisions} and
            d.get('path_state') == 'ACTIVE' for d in diagnostics))


def matching_stopped_completion(item, session, revision, references, completed_state):
    key = (item['session_id'], item['segment_revision'])
    linear, angular = item['linear_speed_mps'], item['angular_speed_radps']
    return (item['state'] == completed_state and item['session_id'] == session and
            item['segment_revision'] == revision and key in references and
            references[key]['reaches_final_goal'] and
            math.isfinite(linear) and math.isfinite(angular) and
            abs(linear) <= .01 and abs(angular) <= .03)


def summarize_rolling_events(events):
    """Count moving replacements and subsequent braking; retain raw events for cause review."""
    moving, stopped, revisions = set(), set(), set()
    previous_tracking = None
    references = {}
    for event in events:
        kind = event.get('kind')
        key = (event.get('session'), event.get('revision'))
        if kind == 'reference':
            revisions.add(key)
            references[key] = event
            if (previous_tracking and previous_tracking.get('session') == key[0]
                    and previous_tracking.get('revision') != key[1]
                    and abs(previous_tracking.get('speed', 0)) > .01):
                moving.add(key)
        elif kind == 'tracking':
            reference = references.get(key, {})
            final_stop = reference.get('final', False) and reference.get('length_m', math.inf) - event.get('progress', 0.) <= .35
            if key in moving and not final_stop and event['phase'] in ('BRAKING', 'ALIGNING'):
                stopped.add(key)
            previous_tracking = event
    return dict(reference_count=len(revisions), moving_updates=len(moving), update_stops=len(stopped))
