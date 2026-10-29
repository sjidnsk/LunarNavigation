"""colcon 包测试：严格线协议不允许 bool 冒充版本整数。"""
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from lunar_nav1cm_verification.contract import decode, envelope, encode, world_from_wire


@pytest.mark.parametrize('version', [True, 2, '1'])
def test_wrong_wire_version_is_rejected(version):
    message = envelope('package-test', 'probe', 'READY', {}, 1., 1)
    message['version'] = version
    with pytest.raises(ValueError):
        decode(json.dumps(message), 'package-test')


def test_unknown_cells_are_not_filled_by_sparse_conversion():
    from nav1cm.contracts import VehicleState
    from nav1cm.maps import extract_window
    world = world_from_wire(dict(epoch=1, resolution_m=.01, blocked=[], known_free=[[-10, -10, 10, 10]]))
    grid = extract_window(world, VehicleState(0, 0, 0, 0, 0), False)
    assert grid.cell_at_world(0, 0) == 0
    assert grid.cell_at_world(1, 1) == -1
