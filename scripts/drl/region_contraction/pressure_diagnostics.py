"""Independent scalar diagnostics for a frozen, possibly running pressure run.

No native/runtime imports and no process control. Only pressure-diagnostics.json
and pressure-diagnostics.csv are written by default. --dry-run writes nothing.
Steps are bounded to the file size at read start. Graph membership containers
are validated/skipped without constructing their Python arrays or dictionaries.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import tempfile


SCHEMA = "region_pressure_diagnostics_v1"
STEP_FIELDS = {"step", "coverage", "distance_m", "new_area_m2", "fine_nodes", "policy_nodes",
    "fine_edges", "policy_edges", "native_actions", "policy_actions", "remote_positions",
    "execution", "reward_parts", "loop_metrics", "timing"}
STEP_IDENTITY_FIELDS = ("actor_sha", "config_sha", "terrain_id", "reference_id", "pairing_digest")
STEP_FIELDS.update(STEP_IDENTITY_FIELDS)
GRAPH_FIELDS = ("fine_nodes", "policy_nodes", "fine_edges", "policy_edges", "native_actions", "policy_actions")
TIMING_FIELDS = ("actor_seconds", "contraction_seconds", "remote_expansion_seconds",
    "privileged_substitution_seconds", "fine_observe_seconds", "env_step_seconds",
    "navigation_step_seconds", "native_execution_wall_seconds")
NONOVERLAPPING = ("actor_seconds", "contraction_seconds", "remote_expansion_seconds",
                 "privileged_substitution_seconds", "env_step_seconds")
LOOP_FIELDS = ("reverse_edge_count", "zero_gain_two_point_loop_steps", "stationary_decisions", "revisit_ratio")
SUMMARY_TIMING_FIELDS = (*TIMING_FIELDS, "reset_wall_seconds", "artifact_seconds", "episode_wall_seconds")
THRESHOLDS = (("at_80", .8), ("at_95", .95), ("at_99", .99))
DECODER = json.JSONDecoder()
NUMBER = re.compile(r"-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?")
WHITESPACE = " \t\r\n"


def _space(text, position):
    while position < len(text) and text[position] in WHITESPACE:
        position += 1
    return position


def _skip_string(text, position):
    position += 1
    while position < len(text):
        char = text[position]
        if char == '"':
            return position + 1
        if ord(char) < 32:
            raise ValueError("control character in JSON string")
        if char == "\\":
            position += 1
            if position >= len(text):
                break
            if text[position] == "u":
                digits = text[position + 1:position + 5]
                if len(digits) != 4 or any(char not in "0123456789abcdefABCDEF" for char in digits):
                    raise ValueError("invalid JSON unicode escape")
                position += 4
            elif text[position] not in '\"\\/bfnrt':
                raise ValueError("invalid JSON string escape")
        position += 1
    raise ValueError("unterminated JSON string")


def _skip_value(text, position, depth=0):
    """Validate JSON grammar while retaining no skipped graph container."""
    if depth > 200:
        raise ValueError("JSON nesting exceeds diagnostic reader bound")
    position = _space(text, position)
    if position >= len(text):
        raise ValueError("missing JSON value")
    char = text[position]
    if char == '"':
        return _skip_string(text, position)
    if char in "[{":
        closing = "]" if char == "[" else "}"
        position = _space(text, position + 1)
        if position < len(text) and text[position] == closing:
            return position + 1
        while True:
            if char == "{":
                if position >= len(text) or text[position] != '"':
                    raise ValueError("JSON object key must be a string")
                position = _space(text, _skip_string(text, position))
                if position >= len(text) or text[position] != ":":
                    raise ValueError("missing JSON key separator")
                position += 1
            position = _space(text, _skip_value(text, position, depth + 1))
            if position >= len(text):
                raise ValueError("unterminated JSON container")
            if text[position] == closing:
                return position + 1
            if text[position] != ",":
                raise ValueError("missing JSON item separator")
            position = _space(text, position + 1)
    for literal in ("true", "false", "null"):
        if text.startswith(literal, position):
            return position + len(literal)
    match = NUMBER.match(text, position)
    if match:
        return match.end()
    raise ValueError("invalid JSON primitive")


def project_step(text):
    """Decode selected top-level fields; large contraction regions are never materialized."""
    position = _space(text, 0)
    if position >= len(text) or text[position] != "{":
        raise ValueError("step record must be a JSON object")
    position = _space(text, position + 1)
    result, seen = {}, set()
    if position < len(text) and text[position] == "}":
        position += 1
    else:
        while True:
            key, end = DECODER.raw_decode(text, position)
            if not isinstance(key, str) or key in seen:
                raise ValueError("invalid or duplicate step field")
            seen.add(key)
            position = _space(text, end)
            if position >= len(text) or text[position] != ":":
                raise ValueError("missing step field separator")
            position = _space(text, position + 1)
            if key in STEP_FIELDS:
                result[key], position = DECODER.raw_decode(text, position)
            else:
                position = _skip_value(text, position)
            position = _space(text, position)
            if position >= len(text):
                raise ValueError("unterminated step object")
            if text[position] == "}":
                position += 1
                break
            if text[position] != ",":
                raise ValueError("missing step field separator")
            position = _space(text, position + 1)
    if _space(text, position) != len(text):
        raise ValueError("trailing data after step object")
    return result


def number(value, name, *, nonnegative=False, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("invalid finite scalar: " + name)
    if nonnegative and value < 0 or integer and int(value) != value:
        raise ValueError("invalid nonnegative/integer scalar: " + name)
    return int(value) if integer else float(value)


def read_json(path, *, optional=False):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("expected JSON object: " + str(path))
        return value
    except FileNotFoundError:
        if optional:
            return None
        raise


def _initial_coverage(directory):
    for filename, key in (("frames/index.json", "frames"), ("observations/index.json", "observations")):
        index = read_json(directory / filename, optional=True)
        if index:
            for row in index.get(key, []):
                if row.get("step") == 0 and row.get("coverage") is not None:
                    coverage = number(row["coverage"], "initial coverage", nonnegative=True)
                    if coverage > 1.000001:
                        raise ValueError("initial coverage outside [0,1]")
                    return coverage
    return None


def _empty_stream():
    return dict(records=0, last_step=None, coverage=None, distance_m=None, initial_distance_m=None,
        graph_peaks={"max_" + name: None for name in GRAPH_FIELDS},
        zero_gain=dict(steps=0, max_consecutive_steps=0, trailing_consecutive_steps=0, ratio=None),
        navigation=dict(navigation_failures=0, collisions=0, reasons={}, **{name: None for name in LOOP_FIELDS}),
        thresholds=dict(at_80=None, at_95=None, at_99=None, steps_80_to_99=None,
            distance_80_to_99_m=None, observed_tail_after_80=None, observed_tail_after_95=None),
        timing={name: dict(count=0, total_seconds=None, mean_seconds=None, max_seconds=None) for name in TIMING_FIELDS},
        partial_tail=False, snapshot_bytes=0, diagnostic_error=None)


def scan_steps(path, initial_coverage=None, identities=None):
    result = _empty_stream()
    path = Path(path)
    if not path.is_file():
        return result
    # Capture an append-only prefix, not an unbounded moving EOF.
    with path.open("rb") as stream:
        remaining = result["snapshot_bytes"] = os.fstat(stream.fileno()).st_size
        previous_distance, previous_coverage = None, None
        while remaining:
            payload = stream.readline(remaining)
            remaining -= len(payload)
            if not payload.endswith(b"\n"):
                result["partial_tail"] = True
                break
            try:
                row = project_step(payload.decode("utf-8"))
                index = number(row["step"], "step", nonnegative=True, integer=True)
                if index != result["records"] + 1:
                    raise ValueError("decisions must start at 1 and increase without gaps")
                coverage = number(row["coverage"], "coverage", nonnegative=True)
                distance = number(row["distance_m"], "distance_m", nonnegative=True)
                gain = number(row["new_area_m2"], "new_area_m2", nonnegative=True)
                if coverage > 1.000001 or previous_coverage is not None and coverage < previous_coverage - 1e-10:
                    raise ValueError("coverage outside [0,1] or nonmonotonic")
                if previous_distance is not None and distance < previous_distance - 1e-8:
                    raise ValueError("nonmonotonic actual distance")
                for name in ("timing", "execution", "loop_metrics", "reward_parts"):
                    if name in row and not isinstance(row[name], dict):
                        raise ValueError(name + " must be a JSON object")
                for name in STEP_IDENTITY_FIELDS:
                    if not isinstance(row.get(name), str) or not row[name]:
                        raise ValueError("missing or invalid step identity: " + name)
                    if (identities or {}).get(name) is not None and row[name] != identities[name]:
                        raise ValueError("step provenance mismatch: " + name)
                graph = {name: number(row[name], name, nonnegative=True, integer=True) for name in GRAPH_FIELDS}
                times = {name: number(value, name, nonnegative=True) for name, value in row.get("timing", {}).items() if name in TIMING_FIELDS}
                loops = {name: number(row["loop_metrics"][name], name, nonnegative=True,
                    integer=name != "revisit_ratio") for name in LOOP_FIELDS if name in row.get("loop_metrics", {})}
                reason = row.get("execution", {}).get("reason_code")
                if reason is not None and not isinstance(reason, str):
                    raise ValueError("execution reason must be a string")
                if index == 1 and row.get("reward_parts", {}).get("distance_m") is not None:
                    action_distance = number(row["reward_parts"]["distance_m"], "action distance", nonnegative=True)
                    initial_distance = distance - action_distance
                    if initial_distance < -1e-8:
                        raise ValueError("action distance exceeds whole-episode distance")
                    result["initial_distance_m"] = max(0., initial_distance)
                # No unselected container remains in row; retain only accumulators.
                result.update(records=index, last_step=index, coverage=coverage, distance_m=distance)
                previous_distance, previous_coverage = distance, coverage
                for name, value in graph.items():
                    key = "max_" + name
                    result["graph_peaks"][key] = value if result["graph_peaks"][key] is None else max(value, result["graph_peaks"][key])
                zero = result["zero_gain"]
                zero["steps"] += int(gain <= 0)
                zero["trailing_consecutive_steps"] = zero["trailing_consecutive_steps"] + 1 if gain <= 0 else 0
                zero["max_consecutive_steps"] = max(zero["max_consecutive_steps"], zero["trailing_consecutive_steps"])
                for label, threshold in THRESHOLDS:
                    if result["thresholds"][label] is None:
                        if initial_coverage is not None and initial_coverage >= threshold:
                            result["thresholds"][label] = dict(step=0, distance_m=result["initial_distance_m"], coverage=initial_coverage)
                        elif coverage >= threshold:
                            result["thresholds"][label] = dict(step=index, distance_m=distance, coverage=coverage)
                nav = result["navigation"]
                if reason is not None:
                    nav["reasons"][reason] = nav["reasons"].get(reason, 0) + 1
                    nav["navigation_failures"] += int(reason != "GOAL_REACHED")
                    nav["collisions"] += int(reason == "COLLISION")
                nav.update(loops)  # These are cumulative geometry counters, not per-step increments.
                for name, value in times.items():
                    item = result["timing"][name]
                    item["count"] += 1
                    item["total_seconds"] = (item["total_seconds"] or 0.) + value
                    item["max_seconds"] = value if item["max_seconds"] is None else max(item["max_seconds"], value)
                del row
            except (ValueError, KeyError, TypeError, UnicodeError) as error:
                result["diagnostic_error"] = f"{path}: record {result['records'] + 1}: {error}"
                break
    if result["records"]:
        result["zero_gain"]["ratio"] = result["zero_gain"]["steps"] / result["records"]
    for item in result["timing"].values():
        if item["count"]:
            item["mean_seconds"] = item["total_seconds"] / item["count"]
    _threshold_difference(result["thresholds"])
    _observed_tail(result)
    return result


def _threshold_difference(thresholds):
    first, last = thresholds["at_80"], thresholds["at_99"]
    if first is not None and last is not None:
        thresholds["steps_80_to_99"] = last["step"] - first["step"]
        if first["distance_m"] is not None and last["distance_m"] is not None:
            thresholds["distance_80_to_99_m"] = last["distance_m"] - first["distance_m"]


def _observed_tail(observed):
    """Endpoint cost observed so far; never fill an unreached 99% endpoint."""
    if observed["last_step"] is None:
        return
    for threshold in (80, 95):
        first = observed["thresholds"][f"at_{threshold}"]
        if first is not None:
            distance = None if first["distance_m"] is None else observed["distance_m"] - first["distance_m"]
            observed["thresholds"][f"observed_tail_after_{threshold}"] = dict(
                steps=observed["last_step"] - first["step"], distance_m=distance,
                through_step=observed["last_step"], through_distance_m=observed["distance_m"],
                through_coverage=observed["coverage"])


def _scan_pose(proof):
    spawn, decision = proof.get("initial_pose"), proof.get("initial_decision_pose")
    result = dict(original_spawn=spawn, initial_decision_pose=decision, yaw_drift_signed_rad=None,
        yaw_drift_abs_rad=None, yaw_drift_signed_deg=None, position_drift_m=None,
        initial_observation_hash=proof.get("initial_observation_hash"),
        exact_initial_observation_hash=proof.get("exact_initial_observation_hash"),
        note="Initialization endpoint versus original scene spawn; timing boundary diagnostic, not a policy action or causal adjustment.")
    if spawn is not None and decision is not None:
        if len(spawn) != 3 or len(decision) != 3:
            raise ValueError("initial pose requires x/y/yaw")
        spawn, decision = [number(value, "spawn pose") for value in spawn], [number(value, "decision pose") for value in decision]
        difference = math.atan2(math.sin(decision[2] - spawn[2]), math.cos(decision[2] - spawn[2]))
        result.update(yaw_drift_signed_rad=difference, yaw_drift_abs_rad=abs(difference),
            yaw_drift_signed_deg=math.degrees(difference), position_drift_m=math.dist(spawn[:2], decision[:2]))
    return result


def _lifecycle(summary, status):
    value = summary or status or {}
    if value.get("infrastructure_failures", 0) or value.get("error_category") == "infrastructure":
        return "infrastructure_error"
    if value.get("error") or value.get("status") == "error":
        return "other_error"
    if summary and summary.get("status") == "finished":
        return "finished"
    return "running" if value.get("status") == "running" else "pending"


def episode(root, case, mode, *, frozen=None, maxima=None):
    root = Path(root).resolve()
    directory = (root / case["name"] / mode).resolve()
    if not directory.is_relative_to(root):
        raise ValueError("case escapes artifact root")
    # Read a terminal record first. A case finishing during this read remains a
    # prefix until the next recomputation, rather than mixing two capture times.
    summary = read_json(directory / "summary.json", optional=True)
    status = read_json(directory / "status.json", optional=True)
    proof = (summary or {}).get("provenance") or read_json(directory / "provenance.json", optional=True) or {}
    initial = _initial_coverage(directory)
    observed = scan_steps(directory / "steps.jsonl", initial, proof)
    lifecycle = _lifecycle(summary, status)
    errors = [observed["diagnostic_error"]] if observed["diagnostic_error"] else []
    final = lifecycle == "finished"
    if final:
        for actual, planned in (("mode", None), ("family", "family"), ("extent_m", "extent"), ("seed", "seed"), ("budget", "budget")):
            expected = mode if actual == "mode" else case[planned]
            if summary.get(actual) != expected:
                errors.append("summary/protocol mismatch: " + actual)
        if summary.get("ended") != 1 or summary.get("decisions") != observed["records"]:
            errors.append("finished summary decisions/ended do not match complete step records")
        if not isinstance(summary.get("completed"), bool):
            errors.append("finished summary requires boolean completed")
        for name in STEP_IDENTITY_FIELDS:
            if not isinstance(proof.get(name), str) or not proof[name]:
                errors.append("finished provenance requires identity: " + name)
        try:
            final_coverage = number(summary.get("final_coverage"), "summary final_coverage", nonnegative=True)
            number(summary.get("distance_m"), "summary distance_m", nonnegative=True)
            number(summary.get("collisions"), "summary collisions", nonnegative=True, integer=True)
            number(summary.get("decisions"), "summary decisions", nonnegative=True, integer=True)
            number(summary.get("ended"), "summary ended", nonnegative=True, integer=True)
            if final_coverage > 1.000001:
                errors.append("finished summary coverage outside [0,1]")
        except ValueError as error:
            errors.append(str(error))
        if observed["partial_tail"]:
            errors.append("finished summary has an incomplete steps tail")
        # Runtime file manifests belong to dispatch, not the episode provenance.
        # Compare the two layers separately rather than pretending a missing
        # episode-level runtime_files manifest is an identity mismatch.
        if frozen and "runtime_files" in frozen:
            dispatch = read_json(directory / "dispatch.json", optional=True) or {}
            if dispatch.get("case") != case or dispatch.get("mode") != mode or dispatch.get("frozen") != frozen:
                errors.append("frozen dispatch identity missing or changed")
        for name, expected in (frozen or {}).items():
            if name == "runtime_files":
                continue
            if proof.get(name) != expected:
                errors.append("frozen provenance mismatch: " + name)
        if observed["records"] and summary.get("final_coverage") != observed["coverage"]:
            errors.append("finished summary coverage differs from final logged step")
        if observed["records"] and summary.get("distance_m") != observed["distance_m"]:
            errors.append("finished summary distance differs from final logged step")
        if observed["records"] == 0 and summary.get("decisions") == 0:
            observed["initial_distance_m"] = summary.get("distance_m")
            for label, threshold in THRESHOLDS:
                if initial is not None and initial >= threshold:
                    observed["thresholds"][label] = dict(step=0, distance_m=summary.get("distance_m"), coverage=initial)
            _threshold_difference(observed["thresholds"])
    final = final and not errors
    nav = dict(observed["navigation"])
    if not observed["records"]:
        nav = {name: None for name in ("navigation_failures", "collisions", *LOOP_FIELDS)} | {"reasons": {}}
    if final:
        for name in ("navigation_failures", "collisions", "reasons", *LOOP_FIELDS):
            if name in summary:
                nav[name] = summary[name]
    timing = dict(steps=observed["timing"], nonoverlapping_decision_seconds=None,
                  final_summary_totals=None, final_accounted_seconds=None, final_unaccounted_seconds=None)
    if observed["records"] and all(observed["timing"][name]["count"] == observed["records"] for name in NONOVERLAPPING):
        timing["nonoverlapping_decision_seconds"] = sum(observed["timing"][name]["total_seconds"] for name in NONOVERLAPPING)
    if final:
        totals = {name: number(value, "summary " + name, nonnegative=True)
                  for name, value in summary.get("timing", {}).items() if name in SUMMARY_TIMING_FIELDS}
        timing["final_summary_totals"] = totals
        components = (*NONOVERLAPPING, "reset_wall_seconds", "artifact_seconds")
        if all(name in totals for name in components):
            timing["final_accounted_seconds"] = sum(totals[name] for name in components)
            if "episode_wall_seconds" in totals:
                timing["final_unaccounted_seconds"] = totals["episode_wall_seconds"] - timing["final_accounted_seconds"]
    resources = dict((maxima or {}).get("cases", {}).get(case["name"] + "/" + mode, {}))
    resources.update(sampled_peak_lower_bound=True,
        note="Resource maxima are sampled process-tree RSS/PSS lower bounds, not exact transient peaks or isolated NN memory. RSS sums count shared pages repeatedly; unavailable PSS stays null.")
    result = dict(case=case["name"], family=case["family"], extent_m=case["extent"], seed=case["seed"],
        budget=case["budget"], mode=mode, lifecycle=lifecycle,
        scope="final" if final else "prefix" if observed["records"] or initial is not None else "unavailable",
        successful=bool(summary.get("completed")) and not summary.get("collisions", 0) if final else None,
        final_coverage=summary.get("final_coverage") if final else None,
        final_decisions=summary.get("decisions") if final else None,
        final_distance_m=summary.get("distance_m") if final else None,
        observed_decisions=observed["records"], prefix_coverage=observed["coverage"] if observed["coverage"] is not None else initial,
        prefix_distance_m=observed["distance_m"], initial_coverage=initial,
        graph_peaks=observed["graph_peaks"], zero_gain=observed["zero_gain"], navigation=nav,
        thresholds=observed["thresholds"], initial_scan=_scan_pose(proof), timing=timing, resources=resources,
        partial_tail=observed["partial_tail"], steps_snapshot_bytes=observed["snapshot_bytes"],
        error=(summary or status or {}).get("error"), error_category=(summary or status or {}).get("error_category"),
        diagnostic_error="; ".join(errors) if errors else None,
        identities={name: proof.get(name) for name in ("actor_sha", "config_sha", "source_commit", "experiment_commit",
            "terrain_id", "reference_id", "pairing_digest", "initial_observation_hash")})
    return result


def collect(root):
    root = Path(root).resolve()
    protocol = read_json(root / "protocol.json")
    cases, modes = protocol["cases"], protocol["modes"]
    if not cases or not modes or len(modes) != len(set(modes)):
        raise ValueError("protocol requires cases and unique modes")
    if len({case["name"] for case in cases}) != len(cases):
        raise ValueError("duplicate protocol case")
    for case in cases:
        if not isinstance(case["name"], str) or Path(case["name"]).name != case["name"] or case["name"] in (".", ".."):
            raise ValueError("case name must be a directory basename")
    if any(mode not in ("baseline", "remote", "contracted") for mode in modes):
        raise ValueError("unsupported protocol mode")
    maxima = read_json(root / "maxima.json", optional=True)
    if maxima and (maxima.get("schema") != "pressure_resource_maxima_v1" or maxima.get("root") != str(root)):
        raise ValueError("resource maxima identity mismatch")
    episodes = []
    for case in cases:
        for mode in modes:
            try:
                row = episode(root, case, mode, frozen=protocol.get("frozen"), maxima=maxima)
            except (ValueError, TypeError, KeyError, OSError) as error:
                row = dict(case=case["name"], family=case["family"], extent_m=case["extent"], seed=case["seed"],
                    budget=case["budget"], mode=mode, scope="unavailable", lifecycle="diagnostic_error",
                    successful=None, final_coverage=None, diagnostic_error=f"{type(error).__name__}: {error}")
            episodes.append(row)
    counts = dict(planned=len(episodes), final_diagnostics=sum(row["scope"] == "final" for row in episodes),
        successes=sum(row["successful"] is True for row in episodes),
        policy_failures=sum(row["successful"] is False for row in episodes),
        infrastructure_errors=sum(row["lifecycle"] == "infrastructure_error" for row in episodes),
        other_errors=sum(row["lifecycle"] == "other_error" for row in episodes),
        diagnostic_errors=sum(row.get("diagnostic_error") is not None for row in episodes),
        prefixes=sum(row["scope"] == "prefix" for row in episodes),
        unavailable=sum(row["scope"] == "unavailable" for row in episodes))
    terminal = counts["final_diagnostics"] + counts["infrastructure_errors"] + counts["other_errors"]
    status = "partial" if terminal != len(episodes) else "final_with_errors" if counts["infrastructure_errors"] + counts["other_errors"] + counts["diagnostic_errors"] else "final"
    return dict(schema=SCHEMA, root=str(root), status=status, counts=counts, episodes=episodes,
        captured_at_utc=datetime.now(timezone.utc).isoformat(),
        collector_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        protocol_sha=hashlib.sha256((root / "protocol.json").read_bytes()).hexdigest(),
        notes=dict(graph_peaks="Exact maxima over complete logged Actor input decisions; pending maxima cover only the prefix. No graph membership or full observations are cached. Cardinality is not measured NN tensor memory.",
            thresholds="First decision endpoint at or above 80%/95%/99%, with its raw accumulated path; intra-action crossing is unknown. 80-to-99 costs require an observed 99% endpoint. observed_tail_after_80/95 is only the cost through the last complete logged decision, including for pending cases. Missing initial frame can hide an already reached initial threshold.",
            timing="Do not sum nested clocks: fine_observe and native_execution_wall overlap env_step; navigation_step is env_step minus fine_observe. Nonoverlapping decision accounting uses actor+contraction+remote_expansion+privileged_substitution+env_step; final accounting additionally uses reset+artifact. Summary fine_observe includes initialization.",
            resources="10-second sampling maxima are lower bounds on process-tree transient peaks. They do not identify isolated NN memory; missing PSS is not replaced by RSS.",
            scope="Only identity-consistent complete summary plus complete matching steps yields final diagnostics. Each admitted step requires all graph cardinalities and Actor/config/terrain/reference/pairing identities matching episode provenance. Running and error episodes preserve prefix evidence and successful=null; no policy-zero imputation or causal claim."))


def _flatten(value, prefix="", output=None):
    output = {} if output is None else output
    for key, item in value.items():
        name = prefix + "." + key if prefix else key
        if isinstance(item, dict):
            _flatten(item, name, output)
        elif isinstance(item, (list, tuple)):
            output[name] = json.dumps(item, allow_nan=False)
        else:
            output[name] = item
    return output


def atomic_file(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_outputs(report, json_path, csv_path):
    for path in (Path(json_path), Path(csv_path)):
        if path.name in ("steps.jsonl", "summary.json", "status.json", "protocol.json", "provenance.json", "maxima.json"):
            raise ValueError("diagnostics cannot replace original evaluation artifacts")
    atomic_file(json_path, (json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode())
    rows = [_flatten(row) for row in report["episodes"]]
    keys = list(dict.fromkeys(key for row in rows for key in row))
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=keys)
    writer.writeheader()
    writer.writerows(rows)  # None becomes an empty CSV cell, not policy zero.
    atomic_file(csv_path, stream.getvalue().encode())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--csv-output", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="read and validate only; write no artifacts")
    args = parser.parse_args(argv)
    report = collect(args.root)
    json_output = args.json_output or args.root / "pressure-diagnostics.json"
    csv_output = args.csv_output or args.root / "pressure-diagnostics.csv"
    if not args.dry_run:
        write_outputs(report, json_output, csv_output)
    print(json.dumps(dict(status=report["status"], counts=report["counts"], dry_run=args.dry_run,
        json_output=str(json_output), csv_output=str(csv_output),
        diagnostic_errors=[dict(case=row["case"], mode=row["mode"], error=row["diagnostic_error"])
                           for row in report["episodes"] if row.get("diagnostic_error")]),
        ensure_ascii=False, allow_nan=False), flush=True)
    return 1 if report["counts"]["diagnostic_errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
