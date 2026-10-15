from pathlib import Path
from lunar_drl_exploration import navigation_identity as identity

def test_every_native_artifact_changes_identity(tmp_path,monkeypatch):
    paths=[]
    for package,relative in [('lunar_incremental_navigation_core','lib/liblunar_incremental_navigation_core.so'),('lunar_incremental_navigation_ros','lib/liblunar_incremental_navigation_ros.so'),('lunar_incremental_navigation_ros','lib/lunar_incremental_navigation_ros/lunar_incremental_navigation_node')]:
        path=tmp_path/package/relative;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'original');paths.append(path)
    monkeypatch.setattr(identity,'get_package_prefix',lambda package:str(tmp_path/package))
    expected=identity.native_navigation_files();assert len(expected)==3
    for index,path in enumerate(paths):
        path.write_bytes(b'new navigation')
        changed=identity.native_navigation_files()
        assert changed[index]!=expected[index]
        assert all(changed[i]==expected[i] for i in range(3) if i!=index)
        path.write_bytes(b'original')
