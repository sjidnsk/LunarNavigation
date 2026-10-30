"""预览命令的产物保护，不加载 Isaac Sim 或执行仿真。"""

import json
from pathlib import Path
import subprocess
import sys


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/isaac_midterm/preview_regions.py"


def run_preview(scene, output):
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--scene", str(scene),
         "--output-dir", str(output), "--dry-run"],
        capture_output=True, text=True, check=False,
    )


def test_existing_render_without_status_is_preserved(tmp_path):
    scene = tmp_path / "scene.usda"
    scene.write_text("#usda 1.0\n", encoding="utf-8")
    output = tmp_path / "previous-preview"
    output.mkdir()
    render = output / "regions-viewport.png"
    original = b"previous-render-must-not-be-overwritten"
    render.write_bytes(original)

    result = run_preview(scene, output)

    assert result.returncode != 0, "含有旧图片的预览目录必须被拒绝"
    assert render.read_bytes() == original
    assert not (output / "stage-status.json").exists()


def test_new_preview_dry_run_does_not_create_directory(tmp_path):
    scene = tmp_path / "scene.usda"
    scene.write_text("#usda 1.0\n", encoding="utf-8")
    output = tmp_path / "new-preview"

    result = run_preview(scene, output)

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload["preview_only"] is True
    assert payload["will_start_simulation"] is False
    assert not output.exists()
