import json
from pathlib import Path
import tempfile
import unittest

from run_p3 import (METHODS, VARIANTS, SUFFIXES, canonical_bytes, historical_projection,
                    prefix, verify_results)


class P3ProtocolTests(unittest.TestCase):
    def test_fifteen_variants_three_repeats_have_unique_prefixes(self):
        names = {prefix(Path("/tmp/p3-test"), method, filter_method, repeat)
                 for method, filter_method in VARIANTS for repeat in range(3)}
        self.assertEqual(len(names), 45)

    def test_historical_projection_excludes_time_but_keeps_counts_and_endpoints(self):
        old = {"method": "baseline", "repeat": 0, "map_ms": 1., "fine_free_count": 1660,
               "start": {"fine_confidence": .4, "raw_available": False}}
        new = {**old, "filter": "none", "map_ms": 99., "start": {**old["start"], "estimate_confidence": .5}}
        reference = canonical_bytes(historical_projection(old, old))
        self.assertEqual(canonical_bytes(historical_projection(new, old)), reference)
        new["fine_free_count"] += 1
        self.assertNotEqual(canonical_bytes(historical_projection(new, old)), reference)
        new["fine_free_count"] -= 1
        new["start"]["fine_confidence"] += .01
        self.assertNotEqual(canonical_bytes(historical_projection(new, old)), reference)

    def test_parity_and_repeat_gate_detect_independent_changes(self):
        # Retain this test's small synthetic files; historical project files
        # and actual comparison outputs are never written or removed here.
        root = Path(tempfile.mkdtemp(prefix="p3-filter-protocol-"))
        historical, output = root/"old", root/"new"
        (historical/"results").mkdir(parents=True); output.mkdir()
        for method, filter_method in VARIANTS:
            for repeat in range(3):
                new = prefix(output, method, filter_method, repeat)
                row = {"method": method, "filter": filter_method, "repeat": repeat,
                       "fine_free_count": 1660, "map_ms": repeat+1., "start": {"fine_confidence": .5}}
                Path(str(new)+"-summary.json").write_text(json.dumps(row))
                for suffix in SUFFIXES: Path(str(new)+suffix).write_bytes(b"same frozen payload")
                if filter_method == "none":
                    old = historical/"results"/(method+"-repeat"+str(repeat))
                    del row["filter"]
                    Path(str(old)+"-summary.json").write_text(json.dumps(row))
                    for suffix in SUFFIXES: Path(str(old)+suffix).write_bytes(b"same frozen payload")
        good = verify_results(output, historical, 3)
        self.assertTrue(good["none_parity_passed"])
        self.assertTrue(good["all_fifteen_variants_repeat_stable"])
        Path(str(prefix(output, "baseline", "median", 2))+"-path.csv").write_bytes(b"changed path")
        changed = verify_results(output, historical, 3)
        self.assertTrue(changed["none_parity_passed"])
        self.assertFalse(changed["all_fifteen_variants_repeat_stable"])
        Path(str(prefix(output, "weighted_plane", "none", 0))+"-ground.bin").write_bytes(b"changed ground")
        self.assertFalse(verify_results(output, historical, 3)["none_parity_passed"])


if __name__ == "__main__": unittest.main()
