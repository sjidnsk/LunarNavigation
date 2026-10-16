"""Read-only localhost pressure dashboard for arbitrary protocol.json cases.

Usage: python3 pressure_serve.py --root ARTIFACT_ROOT --port 39137
This server never starts evaluation, navigation or training processes.
"""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import gzip
import json
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

MODES = ("baseline", "remote")
PAIR_FIELDS = ("actor_sha", "config_sha", "pairing_config_digest", "terrain_id", "reference_id",
               "semantic_id", "polygon_digest", "initial_pose", "pairing_digest", "runtime",
               "source_commit", "source_root", "wrapper_sha", "experiment_sources")


def read_json(path):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def last_jsonl(path):
    """Read a bounded tail, ignoring a partially written final record."""
    try:
        with Path(path).open("rb") as stream:
            stream.seek(0, 2)
            size = stream.tell()
            offset = max(0, size - 4 * 1024 * 1024)
            stream.seek(offset)
            payload = stream.read()
        lines = payload.split(b"\n")
        if offset:
            lines = lines[1:]  # The first line may begin in the middle of JSON.
        lines = lines[:-1]  # Only newline-terminated writes are complete.
        for line in reversed(lines):
            try:
                value = json.loads(line)
                if isinstance(value, dict):
                    return value
            except (ValueError, UnicodeDecodeError):
                continue
    except OSError:
        pass
    return None


def classification(summary, status):
    value = summary or status or {}
    if value.get("infrastructure_failures", 0) or value.get("error_category") == "infrastructure":
        return "infrastructure"
    if value.get("error") or value.get("status") == "error":
        return "error"
    if summary and summary.get("status") == "finished" and summary.get("ended", 1):
        return "finished"
    if value.get("status") == "running":
        return "running"
    return "pending"


def counts(records):
    result = dict(planned=len(records), terminal=0, ended=0, successes=0,
                  infrastructure=0, other_errors=0, running=0, pending=0)
    coverages = []
    for record in records:
        kind = record["classification"]
        summary = record.get("summary") or {}
        if kind in ("finished", "infrastructure", "error"):
            result["terminal"] += 1
        if kind == "finished":
            result["ended"] += 1
            result["successes"] += int(bool(summary.get("completed")) and not summary.get("collisions", 0))
            value = summary.get("final_coverage")
            if isinstance(value, (int, float)):
                coverages.append(value)
        elif kind == "infrastructure":
            result["infrastructure"] += 1
        elif kind == "error":
            result["other_errors"] += 1
        else:
            result[kind] += 1
    result["success_rate"] = result["successes"] / result["ended"] if result["ended"] else None
    result["mean_coverage"] = sum(coverages) / len(coverages) if coverages else None
    return result


def pair_status(records):
    if len(records) != len(MODES) or any(record["classification"] != "finished" for record in records):
        return dict(status="incomplete", differences=[])
    summaries = [record.get("summary") or {} for record in records]
    proofs = [summary.get("provenance") or {} for summary in summaries]
    missing = [key for key in PAIR_FIELDS if any(proof.get(key) in (None, "") for proof in proofs)]
    differing = [key for key in PAIR_FIELDS if proofs[0].get(key) != proofs[1].get(key)]
    for key in ("family", "extent_m", "seed", "budget", "success_coverage"):
        if any(summary.get(key) is None for summary in summaries):
            missing.append(key)
        if summaries[0].get(key) != summaries[1].get(key):
            differing.append(key)
    for record, summary in zip(records, summaries):
        case = record["case"]
        expected = dict(family=case.get("family"), extent_m=case.get("extent", case.get("extent_m")),
                        seed=case.get("seed"), budget=case.get("budget"), mode=record["mode"])
        for key, value in expected.items():
            if summary.get(key) != value and key not in differing:
                differing.append(key)
    return dict(status="mismatch" if differing else "missing_identity" if missing else "matched",
                differences=differing, missing=missing)


class ArtifactStore:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.tail_cache = {}
        self.frame_cache = {}
        self.lock = threading.Lock()

    def case_root(self, name, mode):
        if not isinstance(name, str) or not name or "/" in name or "\\" in name or name in (".", ".."):
            return None
        result = (self.root / name / mode).resolve()
        return result if result.is_relative_to(self.root) else None

    def latest_step(self, path):
        try:
            info = path.stat()
            identity = (info.st_size, info.st_mtime_ns)
        except OSError:
            return None
        with self.lock:
            cached = self.tail_cache.get(str(path))
            if cached and cached[0] == identity:
                return cached[1]
            value = last_jsonl(path)
            if value is not None:
                # Large contraction membership lists do not belong in overview polling.
                value = {key: value[key] for key in ("step", "coverage", "fine_nodes", "policy_nodes",
                    "fine_edges", "policy_edges", "native_actions", "policy_actions", "remote_positions",
                    "new_area_m2", "distance_m", "timing", "loop_metrics", "progress", "selected_goal") if key in value}
            self.tail_cache[str(path)] = (identity, value)
            return value

    def latest_frame(self, directory):
        index = read_json(directory / "frames/index.json") or {}
        frames = index.get("frames", [])
        if not frames:
            return None
        relative = frames[-1].get("path", "")
        path = (directory / relative).resolve()
        if not path.is_relative_to(directory) or not path.is_file():
            return None
        info = path.stat()
        identity = (str(path), info.st_size, info.st_mtime_ns)
        with self.lock:
            cached = self.frame_cache.get(str(directory))
            if cached and cached[0] == identity:
                return cached[1]
            try:
                frame = json.loads(gzip.decompress(path.read_bytes()))
                result = dict(step=frame.get("step"), coverage=frame.get("coverage"),
                    fine_nodes=len(frame.get("fine_nodes", [])), policy_nodes=len(frame.get("coarse_nodes", [])),
                    fine_edges=len(frame.get("fine_edges", [])), policy_edges=len(frame.get("coarse_edges", [])),
                    virtual_nodes=sum(frame.get("virtual_mask", [])), actions=len(frame.get("action_goals", [])))
                self.frame_cache[str(directory)] = (identity, result)
                return result
            except (OSError, ValueError, EOFError):
                return None

    def overview(self):
        protocol = read_json(self.root / "protocol.json")
        records, warnings = [], []
        for case in (protocol or {}).get("cases", []):
            if not isinstance(case, dict):
                warnings.append("Ignored non-object protocol case")
                continue
            for mode in MODES:
                directory = self.case_root(case.get("name"), mode)
                if directory is None:
                    warnings.append("Ignored unsafe case name")
                    continue
                summary = read_json(directory / "summary.json")
                status = read_json(directory / "status.json")
                latest = self.latest_step(directory / "steps.jsonl")
                kind = classification(summary, status)
                records.append(dict(case=case, mode=mode, summary=summary, status=status, latest_step=latest,
                    latest_frame=self.latest_frame(directory) if kind == "running" else None, classification=kind))
        groups = []
        extents = sorted({float(record["case"].get("extent", record["case"].get("extent_m", 0))) for record in records})
        for extent in extents:
            subset = [record for record in records if float(record["case"].get("extent", record["case"].get("extent_m", 0))) == extent]
            groups.append(dict(extent_m=extent, modes={mode: counts([record for record in subset if record["mode"] == mode])
                                                       for mode in MODES}))
        pairs = {case["name"]: pair_status([record for record in records if record["case"]["name"] == case["name"]])
                 for case in (protocol or {}).get("cases", []) if isinstance(case, dict) and isinstance(case.get("name"), str)}
        return dict(schema="region_pressure_overview_v1", updated_unix=time.time(),
            protocol=protocol, run_state=read_json(self.root / "run-state.json"),
            provenance=read_json(self.root / "provenance.json"), records=records,
            overall=counts(records), modes={mode: counts([record for record in records if record["mode"] == mode]) for mode in MODES},
            groups=groups, pairs=pairs, warnings=warnings)


class PressureHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, store, **kwargs):
        self.store = store
        super().__init__(*args, directory=str(store.root), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        if urlsplit(self.path).path == "/api/overview":
            payload = (json.dumps(self.store.overview(), ensure_ascii=False, allow_nan=False) + "\n").encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        super().do_GET()

    def translate_path(self, path):
        resource = urlsplit(path).path
        if resource in ("/", "/index.html", "/pressure.html"):
            return str(Path(__file__).with_name("pressure_viewer.html"))
        if resource.startswith("/data/"):
            path = resource[5:]
        result = Path(super().translate_path(path)).resolve()
        if not result.is_relative_to(self.store.root):
            return str(self.store.root / ".invalid-pressure-path")
        return str(result)

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--port", type=int, default=39137)
    args = parser.parse_args()
    if not args.root.is_dir():
        parser.error("artifact root must be an existing directory")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(PressureHandler, store=ArtifactStore(args.root)))
    print(f"http://127.0.0.1:{server.server_port}/pressure.html", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
