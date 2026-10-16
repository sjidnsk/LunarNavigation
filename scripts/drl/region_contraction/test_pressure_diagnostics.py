"""Pure-data regression tests for the independent pressure diagnostic reader."""
import importlib
import json
import math
from pathlib import Path

import pytest


def implementation():
    try:
        return importlib.import_module("scripts.drl.region_contraction.pressure_diagnostics")
    except ModuleNotFoundError as error:
        if error.name.endswith("pressure_diagnostics"):
            pytest.fail("independent pressure diagnostics reader is not implemented")
        raise


def step(index, coverage, distance, gain, *, fine=10, policy=7, timing=None, reason="GOAL_REACHED"):
    return dict(step=index, coverage=coverage, distance_m=distance, new_area_m2=gain,
        fine_nodes=fine, policy_nodes=policy, fine_edges=fine * 2, policy_edges=policy * 2,
        native_actions=8, policy_actions=16,
        actor_sha="actor", config_sha="config", terrain_id="terrain", reference_id="reference", pairing_digest="pair",
        execution=dict(reason_code=reason, outcome=6 if reason == "COLLISION" else 0),
        reward_parts=dict(distance_m=distance if index == 1 else 1.),
        loop_metrics=dict(reverse_edge_count=index, zero_gain_two_point_loop_steps=index - 1,
                          stationary_decisions=0, revisit_ratio=.25),
        timing=timing or {}, contraction=dict(regions=[dict(member_indices=list(range(100))) for _ in range(5)]))


def fixture(tmp_path, rows=(), *, summary=None, initial_coverage=.2):
    case = dict(name="moon100-r1", family="moon", extent=100, seed=77, budget=512)
    root = tmp_path
    (root / "protocol.json").write_text(json.dumps(dict(cases=[case], modes=["baseline", "remote"])))
    directory = root / case["name"] / "baseline"
    directory.mkdir(parents=True)
    (directory / "steps.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    proof = dict(initial_pose=[1., 2., math.pi - .1],
        initial_decision_pose=[1., 2., -math.pi + .2], initial_observation_hash="observed",
        exact_initial_observation_hash="transport", pairing_digest="pair", actor_sha="actor", config_sha="config",
        terrain_id="terrain", reference_id="reference")
    (directory / "provenance.json").write_text(json.dumps(proof))
    (directory / "frames").mkdir()
    (directory / "frames/index.json").write_text(json.dumps(dict(frames=[dict(step=0, coverage=initial_coverage)])))
    if summary is not None:
        if "provenance" in summary:
            summary["provenance"] = proof | summary["provenance"]
        (directory / "summary.json").write_text(json.dumps(summary))
    return root, case, directory


def finished(count, *, successful=True, distance=30):
    return dict(status="finished", mode="baseline", family="moon", extent_m=100, seed=77,
        budget=512, ended=1, completed=successful, decisions=count, distance_m=distance,
        final_coverage=.995 if successful else .9, error=None, collisions=0,
        navigation_failures=0, reverse_edge_count=4, zero_gain_two_point_loop_steps=3,
        stationary_decisions=0, revisit_ratio=.25,
        timing=dict(actor_seconds=4., contraction_seconds=8., remote_expansion_seconds=12.,
            privileged_substitution_seconds=16., env_step_seconds=40., fine_observe_seconds=17.,
            navigation_step_seconds=24., native_execution_wall_seconds=36.,
            reset_wall_seconds=3., artifact_seconds=2., episode_wall_seconds=90.))


def test_streaming_projection_does_not_return_membership_and_validates_skipped_json():
    module = implementation()
    source = step(1, .2, 1., 0.)
    source["irrelevant"] = {"escaped": "brackets [ ] { } and \"quotes\"", "values": [None, True, -1.2e4]}
    row = module.project_step(json.dumps(source))
    assert row["fine_nodes"] == 10 and row["policy_nodes"] == 7
    assert "contraction" not in row and "irrelevant" not in row
    with pytest.raises(ValueError):
        module.project_step('{"step":1,"contraction":{"members":[1,]}}')


def test_actual_graph_peaks_zero_runs_and_threshold_endpoints(tmp_path):
    module = implementation()
    rows = [step(1, .5, 1., 0., fine=10, policy=8),
            step(2, .81, 10., 0., fine=100, policy=50),
            step(3, .9, 20., 5., fine=30, policy=20),
            step(4, .995, 30., 0., fine=40, policy=25)]
    root, case, _ = fixture(tmp_path, rows, summary=finished(4))
    result = module.episode(root, case, "baseline")
    assert result["scope"] == "final" and result["successful"] is True
    assert result["graph_peaks"]["max_fine_nodes"] == 100
    assert result["graph_peaks"]["max_policy_nodes"] == 50
    assert result["graph_peaks"]["max_fine_edges"] == 200
    assert result["graph_peaks"]["max_policy_edges"] == 100
    assert result["zero_gain"]["max_consecutive_steps"] == 2
    assert result["zero_gain"]["steps"] == 3
    assert result["thresholds"]["at_80"]["step"] == 2
    assert result["thresholds"]["at_99"]["step"] == 4
    assert result["thresholds"]["steps_80_to_99"] == 2
    assert result["thresholds"]["distance_80_to_99_m"] == 20.
    assert result["navigation"]["reverse_edge_count"] == 4  # Cumulative counter, never sum 1+2+3+4.
    assert "member_indices" not in json.dumps(result)


def test_initially_reached_80_uses_step_zero_and_actual_initial_distance(tmp_path):
    module = implementation()
    rows = [step(1, .995, 5., 2.)]
    root, case, directory = fixture(tmp_path, rows, summary=finished(1, distance=5), initial_coverage=.85)
    result = module.episode(root, case, "baseline")
    assert result["thresholds"]["at_80"] == dict(step=0, distance_m=0., coverage=.85)
    assert result["thresholds"]["steps_80_to_99"] == 1
    assert result["thresholds"]["distance_80_to_99_m"] == 5.


def test_overlapping_timers_are_not_added_to_core_accounting(tmp_path):
    module = implementation()
    timing = dict(actor_seconds=1., contraction_seconds=2., remote_expansion_seconds=3.,
        privileged_substitution_seconds=4., env_step_seconds=10., fine_observe_seconds=4.,
        navigation_step_seconds=6., native_execution_wall_seconds=9.)
    rows = [step(i, .5, i, 1., timing=timing) for i in range(1, 5)]
    summary = finished(4, successful=False, distance=4)
    summary["final_coverage"] = .5
    root, case, _ = fixture(tmp_path, rows, summary=summary)
    result = module.episode(root, case, "baseline")
    assert result["timing"]["steps"]["env_step_seconds"]["total_seconds"] == 40.
    assert result["timing"]["steps"]["fine_observe_seconds"]["mean_seconds"] == 4.
    assert result["timing"]["nonoverlapping_decision_seconds"] == 80.
    assert result["timing"]["final_accounted_seconds"] == 85.  # 80 + reset 3 + artifact 2.
    assert result["timing"]["final_unaccounted_seconds"] == 5.


def test_scan_yaw_drift_wraps_around_pi_without_changing_spawn_semantics(tmp_path):
    module = implementation()
    root, case, _ = fixture(tmp_path, [step(1, .4, 1., 2.)])
    result = module.episode(root, case, "baseline")
    assert result["initial_scan"]["yaw_drift_signed_rad"] == pytest.approx(.3)
    assert result["initial_scan"]["position_drift_m"] == 0.
    assert result["scope"] == "prefix" and result["successful"] is None


def test_pending_and_infrastructure_error_never_become_policy_zero(tmp_path):
    module = implementation()
    root, case, directory = fixture(tmp_path, [step(1, .4, 1., 0.)],
        summary=dict(status="error", error_category="infrastructure", error="worker disconnected", ended=0))
    result = module.episode(root, case, "baseline")
    assert result["lifecycle"] == "infrastructure_error" and result["scope"] == "prefix"
    assert result["successful"] is None and result["final_coverage"] is None
    assert result["prefix_coverage"] == .4
    pending = module.episode(root, case, "remote")
    assert pending["scope"] == "unavailable" and pending["successful"] is None
    assert pending["graph_peaks"]["max_fine_nodes"] is None


def test_missing_finished_records_are_explicitly_not_final(tmp_path):
    module = implementation()
    root, case, _ = fixture(tmp_path, [step(1, .4, 1., 2.)], summary=finished(2))
    result = module.episode(root, case, "baseline")
    assert result["scope"] == "prefix" and result["successful"] is None
    assert "decisions" in result["diagnostic_error"]


def test_running_partial_tail_is_skipped_but_malformed_complete_line_is_error(tmp_path):
    module = implementation()
    root, case, directory = fixture(tmp_path, [step(1, .4, 1., 2.)])
    with (directory / "steps.jsonl").open("a") as stream:
        stream.write('{"step":2')
    result = module.episode(root, case, "baseline")
    assert result["observed_decisions"] == 1 and result["partial_tail"]
    (directory / "steps.jsonl").write_text('{"step":1,}\n')
    result = module.episode(root, case, "baseline")
    assert result["diagnostic_error"] and result["scope"] == "prefix"
    assert result["observed_decisions"] == 0 and result["successful"] is None


def test_not_reached_99_keeps_raw_95_and_only_observed_finishing_cost(tmp_path):
    module = implementation()
    rows = [step(1, .81, 10., 3.), step(2, .9623, 20., 4.), step(3, .9651, 35., 0.)]
    root, case, _ = fixture(tmp_path, rows)
    thresholds = module.episode(root, case, "baseline")["thresholds"]
    assert thresholds["at_80"] == dict(step=1, distance_m=10., coverage=.81)
    assert thresholds["at_95"] == dict(step=2, distance_m=20., coverage=.9623)
    assert thresholds["at_99"] is None
    assert thresholds["steps_80_to_99"] is None and thresholds["distance_80_to_99_m"] is None
    assert thresholds["observed_tail_after_80"] == dict(steps=2, distance_m=25.,
        through_step=3, through_distance_m=35., through_coverage=.9651)
    assert thresholds["observed_tail_after_95"] == dict(steps=1, distance_m=15.,
        through_step=3, through_distance_m=35., through_coverage=.9651)


def test_missing_completed_cannot_impute_final_policy_failure(tmp_path):
    module = implementation()
    summary = finished(1, successful=False, distance=1.)
    del summary["completed"]
    root, case, _ = fixture(tmp_path, [step(1, .9, 1., 2.)], summary=summary)
    result = module.episode(root, case, "baseline")
    assert result["diagnostic_error"] and result["scope"] == "prefix"
    assert result["successful"] is None


def test_valid_completed_failure_remains_a_policy_failure(tmp_path):
    module = implementation()
    root, _, _ = fixture(tmp_path, [step(1, .9, 1., 2.)],
        summary=finished(1, successful=False, distance=1.))
    report = module.collect(root)
    assert report["episodes"][0]["scope"] == "final"
    assert report["episodes"][0]["successful"] is False
    assert report["counts"]["policy_failures"] == 1


def test_final_distance_must_match_complete_logged_path(tmp_path):
    module = implementation()
    root, case, _ = fixture(tmp_path, [step(1, .995, 1., 2.)], summary=finished(1, distance=50.))
    result = module.episode(root, case, "baseline")
    assert result["scope"] == "prefix" and result["successful"] is None
    assert "distance" in result["diagnostic_error"]


@pytest.mark.parametrize("field", ["fine_nodes", "policy_nodes", "fine_edges", "policy_edges", "native_actions", "policy_actions"])
def test_missing_required_graph_count_cannot_become_a_final_peak(tmp_path, field):
    module = implementation()
    rows = [step(1, .8, 1., 1.), step(2, .995, 2., 1., fine=10000)]
    del rows[1][field]
    root, case, _ = fixture(tmp_path, rows, summary=finished(2, distance=2.))
    result = module.episode(root, case, "baseline")
    assert result["scope"] == "prefix" and result["successful"] is None
    assert result["observed_decisions"] == 1 and field in result["diagnostic_error"]


@pytest.mark.parametrize("field", ["actor_sha", "config_sha", "terrain_id", "reference_id", "pairing_digest"])
def test_step_identity_mismatch_is_not_hidden_by_correct_summary(tmp_path, field):
    module = implementation()
    rows = [step(1, .8, 1., 1.), step(2, .995, 2., 1.)]
    rows[1][field] = "another episode"
    root, case, _ = fixture(tmp_path, rows, summary=finished(2, distance=2.))
    result = module.episode(root, case, "baseline")
    assert result["scope"] == "prefix" and result["successful"] is None
    assert result["observed_decisions"] == 1 and field in result["diagnostic_error"]


def test_runtime_manifest_validates_dispatch_separately_and_does_not_hash_live_source(tmp_path):
    module = implementation()
    frozen = dict(actor_sha="actor", runtime=dict(modules={}), runtime_files={"/does/not/exist": "frozenhash"})
    summary = finished(1, distance=1.)
    summary["provenance"] = dict(actor_sha="actor", runtime=dict(modules={}))
    root, case, directory = fixture(tmp_path, [step(1, .995, 1., 2.)], summary=summary)
    dispatch = dict(case=case, mode="baseline", frozen=json.loads(json.dumps(frozen)))
    (directory / "dispatch.json").write_text(json.dumps(dispatch))
    assert module.episode(root, case, "baseline", frozen=frozen)["scope"] == "final"
    dispatch["frozen"]["actor_sha"] = "changed"
    (directory / "dispatch.json").write_text(json.dumps(dispatch))
    result = module.episode(root, case, "baseline", frozen=frozen)
    assert result["successful"] is None and "dispatch" in result["diagnostic_error"]


def test_finished_at_initial_scan_has_zero_decisions_without_invented_graph_peaks(tmp_path):
    module = implementation()
    root, case, _ = fixture(tmp_path, summary=finished(0, distance=0.), initial_coverage=.995)
    result = module.episode(root, case, "baseline")
    assert result["scope"] == "final" and result["successful"] is True
    assert result["graph_peaks"]["max_fine_nodes"] is None
    assert result["thresholds"]["at_95"] == dict(step=0, distance_m=0., coverage=.995)
    assert result["thresholds"]["steps_80_to_99"] == 0
    assert result["thresholds"]["observed_tail_after_95"] is None


@pytest.mark.parametrize("changes", [dict(step=0), dict(coverage=float("nan")), dict(distance_m=-1), dict(timing=dict(env_step_seconds=-1))])
def test_invalid_scalar_is_not_silently_aggregated(tmp_path, changes):
    module = implementation()
    row = step(1, .4, 1., 2.)
    row.update(changes)
    root, case, _ = fixture(tmp_path, [row])
    assert module.episode(root, case, "baseline")["diagnostic_error"]


@pytest.mark.parametrize("field,value", [("timing", []), ("execution", 1), ("loop_metrics", None), ("reward_parts", "bad")])
def test_invalid_small_container_marks_case_error_without_crashing_collection(tmp_path, field, value):
    module = implementation()
    row = step(1, .4, 1., 2.)
    row[field] = value
    root, _, _ = fixture(tmp_path, [row])
    report = module.collect(root)
    assert report["counts"]["diagnostic_errors"] == 1
    assert report["episodes"][0]["successful"] is None
    assert field in report["episodes"][0]["diagnostic_error"]


def test_resource_maxima_remain_sampled_lower_bounds_and_csv_keeps_null(tmp_path):
    module = implementation()
    root, case, _ = fixture(tmp_path, [step(1, .4, 1., 2.)])
    (root / "maxima.json").write_text(json.dumps(dict(schema="pressure_resource_maxima_v1", root=str(root.resolve()),
        cases={case["name"] + "/baseline": dict(samples=2, complete_samples=1, rss_max_kib=2048, pss_max_kib=None)})))
    report = module.collect(root)
    row = report["episodes"][0]
    assert row["resources"]["sampled_peak_lower_bound"] is True
    assert row["resources"]["rss_max_kib"] == 2048 and row["resources"]["pss_max_kib"] is None
    out_json, out_csv = tmp_path / "diagnostics.json", tmp_path / "diagnostics.csv"
    module.write_outputs(report, out_json, out_csv)
    import csv
    records = list(csv.DictReader(out_csv.open()))
    assert records[0]["successful"] == "" and records[0]["final_coverage"] == ""
    assert records[0]["graph_peaks.max_fine_nodes"] == "10"
    assert json.loads(out_json.read_text())["status"] == "partial"
