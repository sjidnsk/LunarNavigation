from copy import deepcopy
import pytest
from tools.compare_search_optimization import equivalent, read


def test_accepts_different_timing_but_identical_path():
    left = [{"id": "0", "status": "0", "path": "0x1p+0", "ms": "10"}]
    right = deepcopy(left)
    right[0]["ms"] = "2"
    assert equivalent(left, right, ["id"], ["status", "path"]) == 1


@pytest.mark.parametrize("field", ["status", "path"])
def test_rejects_changed_result_even_when_timing_improves(field):
    left = [{"id": "0", "status": "0", "path": "0x1p+0"}]
    right = deepcopy(left)
    right[0][field] = "different"
    with pytest.raises(ValueError, match="Output mismatch"):
        equivalent(left, right, ["id"], ["status", "path"])


def test_rejects_missing_or_duplicate_case():
    row = {"id": "0", "path": "same"}
    with pytest.raises(ValueError, match="keys differ"):
        equivalent([row], [], ["id"], ["path"])
    with pytest.raises(ValueError, match="Duplicate"):
        equivalent([row], [row, row], ["id"], ["path"])


def test_rejects_incomplete_csv(tmp_path):
    path = tmp_path / "partial.csv"
    path.write_text("id,path\n0,same\n")
    with pytest.raises(ValueError, match="expected 2 rows"):
        read(path, 2)
