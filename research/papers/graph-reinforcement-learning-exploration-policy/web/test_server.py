"""仅在 D 盘复制 fixture 中验证 HTTP 行为；保留产物便于检查。"""
import hashlib
import http.client
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
import server

SOURCE = Path(__file__).resolve().parent.parent
ARTIFACTS = server.DEFAULT_ARTIFACTS
COPY_FILES = ["README.md"] + ["02-research-design/" + name for name in
              ("progress.py", "work-items.json", "PROGRESS.md", "CHECKLIST.md", "WORKPLAN.md")]


def fingerprint(root):
    return {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in COPY_FILES}


class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source_before = fingerprint(SOURCE)
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        cls.test_root = Path(tempfile.mkdtemp(prefix="test-backend-", dir=ARTIFACTS))
        print(f"\n测试产物：{cls.test_root}")

    @classmethod
    def tearDownClass(cls):
        if fingerprint(SOURCE) != cls.source_before:
            raise AssertionError("源码工作树中的研究资料发生变化")

    def setUp(self):
        self.fixture = Path(tempfile.mkdtemp(prefix="case-", dir=self.test_root))
        self.paper = self.fixture / "paper"
        for name in COPY_FILES:
            destination = self.paper / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(SOURCE / name, destination)
        self.httpd = server.make_server(self.paper, backup_dir=self.fixture / "backups")
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        self.origin = f"http://127.0.0.1:{self.httpd.server_port}"

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=3)

    def request(self, method, path, payload=None, headers=None, raw=None):
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8") if payload is not None else raw
        actual = {"Origin": self.origin, "Content-Type": "application/json"}
        if headers:
            actual.update(headers)
        connection = http.client.HTTPConnection("127.0.0.1", self.httpd.server_port, timeout=5)
        try:
            connection.request(method, path, body=content, headers=actual)
            response = connection.getresponse()
            body = response.read()
            if "application/json" in response.getheader("Content-Type", ""):
                return response.status, json.loads(body)
            return response.status, body
        finally:
            connection.close()

    def workspace(self):
        status, value = self.request("GET", "/api/workspace")
        self.assertEqual(status, 200, value)
        return value

    def change(self, key="P0-01", extra=None, suffix="", expected=200, workspace=None):
        ws = workspace or self.workspace()
        payload = {"revision": ws["revision"], "csrf_token": ws["csrf_token"]}
        payload.update(extra or {})
        status, result = self.request("POST", f"/api/tasks/{key}{suffix}", payload)
        self.assertEqual(status, expected, result)
        return result

    def task(self, workspace, key="P0-01"):
        return next(t for t in workspace["ledger"]["tasks"] if t["id"] == key)

    def originals(self):
        return {path: path.read_bytes() for path in self.httpd.workspace.paths}

    def test_initialization_is_read_only_and_documents_allowlisted(self):
        self.assertEqual(fingerprint(self.paper), self.source_before)
        ws = self.workspace()
        self.assertEqual(len(ws["ledger"]["tasks"]), 61)
        self.assertEqual(self.task(ws, "P3-01")["status"], "doing")
        self.assertEqual(len(ws["ready_ids"]), len(self.httpd.workspace.progress.ready(ws["ledger"])))
        self.assertEqual(ws["pending_dependencies"]["P0-02"], ["P0-01"])
        for document in ws["documents"]:
            status, body = self.request("GET", "/api/documents/" + document["id"])
            self.assertEqual(status, 200)
            self.assertTrue(body["markdown"])
        self.assertEqual(self.request("GET", "/health")[1]["status"], "ok")
        self.assertEqual(fingerprint(self.paper), self.source_before)
        self.assertFalse((self.fixture / "backups").exists())

    def test_update_synchronizes_all_files_and_retains_backup(self):
        old = self.originals()
        ws = self.change(extra={"status": "doing", "owner": "测试作者", "due": "2026-11-01", "note": "隔离测试"})
        task = self.task(ws)
        self.assertEqual(task["status"], "doing")
        self.assertEqual(task["owner"], "测试作者")
        self.assertEqual(task["history"][-1]["to"], "doing")
        for path in self.httpd.workspace.paths:
            self.assertNotEqual(path.read_bytes(), old[path])
        self.assertIn("测试作者", self.httpd.workspace.paths[1].read_text(encoding="utf-8"))
        self.assertIn("隔离测试", self.httpd.workspace.paths[2].read_text(encoding="utf-8"))
        backups = list((self.fixture / "backups").glob("save-*"))
        self.assertEqual(len(backups), 1)
        for path in old:
            self.assertEqual((backups[0] / path.name).read_bytes(), old[path])

    def test_full_evidence_edit_and_due_clear_record_history(self):
        self.change(extra={"evidence": ["旧证据", "保留证据"], "due": "2026-11-01"})
        ws = self.change(extra={"evidence": ["保留证据", "新证据"], "due": "", "note": "完整编辑"})
        task = self.task(ws)
        self.assertEqual(task["evidence"], ["保留证据", "新证据"])
        self.assertIsNone(task["due_on"])
        self.assertEqual(task["history"][-1]["evidence"], task["evidence"])
        self.assertIsNone(task["history"][-1]["due_on"])
        ws = self.change(extra={"evidence": [], "note": ""})
        self.assertEqual(self.task(ws)["evidence"], [])
        self.assertEqual(self.task(ws)["note"], "")

    def test_dependency_rejection_and_no_writes(self):
        before = self.originals()
        for status in ("doing", "done"):
            self.change("P0-02", {"status": status, "note": "测试", "evidence": ["fixture"]}, expected=400)
        self.assertEqual(before, self.originals())

    def test_completion_requires_evidence_and_note(self):
        for extra in ({"status": "done", "note": "验收"},
                      {"status": "done", "evidence": ["fixture"]},
                      {"status": "done", "note": " ", "evidence": ["fixture"]}):
            self.change(extra=extra, expected=400)
        ws = self.change(extra={"status": "done", "note": "验收记录", "evidence": ["fixture"]})
        self.assertEqual(self.task(ws)["status"], "done")
        self.change(extra={"evidence": []}, expected=400)
        self.change(extra={"status": "todo"}, expected=400)

    def test_waiver_gate_and_authorization_guards(self):
        ws = self.workspace()
        gate = next(t for t in ws["ledger"]["tasks"] if t.get("kind") == "gate")
        authorization = next(t for t in ws["ledger"]["tasks"] if "授权" in t["title"] and not t.get("can_waive"))
        for key in (gate["id"], authorization["id"], "P0-01"):
            self.change(key, {"status": "waived", "note": "作者决定", "evidence": ["fixture"]}, expected=400)
        conditional = next(t for t in ws["ledger"]["tasks"] if t.get("can_waive"))
        changed = self.change(conditional["id"], {"status": "waived", "note": "隔离范围收缩决定", "evidence": ["fixture"]})
        self.assertEqual(self.task(changed, conditional["id"])["status"], "waived")

    def test_reopen_invalidates_closed_downstream(self):
        self.change(extra={"status": "done", "note": "验收", "evidence": ["fixture"]})
        self.change("P0-02", {"status": "done", "note": "验收", "evidence": ["fixture"]})
        self.change(extra={"reason": " "}, suffix="/reopen", expected=400)
        ws = self.change(extra={"reason": "证据需要复核"}, suffix="/reopen")
        self.assertEqual(self.task(ws)["status"], "todo")
        self.assertEqual(self.task(ws, "P0-02")["status"], "review")
        self.assertIsNone(self.task(ws, "P0-02")["completed_on"])
        self.assertIn("证据需要复核", self.task(ws, "P0-02")["note"])

    def test_revision_detects_other_page_and_external_changes(self):
        stale = self.workspace()
        self.change(extra={"owner": "页面一"})
        self.change(extra={"owner": "页面二"}, workspace=stale, expected=409)
        stale = self.workspace()
        path = self.httpd.workspace.paths[0]
        path.write_bytes(path.read_bytes() + b"\n")
        self.change(extra={"owner": "外部更改后"}, workspace=stale, expected=409)

    def test_same_origin_host_and_token_required(self):
        ws = self.workspace()
        payload = {"revision": ws["revision"], "csrf_token": ws["csrf_token"], "owner": "测试"}
        for headers in ({"Origin": "http://evil.example"}, {"Origin": "null"},
                        {"Host": "evil.example"}, {"Sec-Fetch-Site": "cross-site"}):
            self.assertEqual(self.request("POST", "/api/tasks/P0-01", payload, headers=headers)[0], 403)
        for token in ("错误令牌", "\\ud800", None, 7, ""):
            payload["csrf_token"] = token
            self.assertEqual(self.request("POST", "/api/tasks/P0-01", payload)[0], 403)
        self.assertEqual(self.request("GET", "/api/workspace", headers={"Host": "evil.example"})[0], 403)
        # 无 Origin 的标准库 GET 被允许；写请求必须有 Origin。
        connection = http.client.HTTPConnection("127.0.0.1", self.httpd.server_port)
        connection.request("POST", "/api/tasks/P0-01", body=json.dumps(payload), headers={"Content-Type": "application/json"})
        response = connection.getresponse()
        self.assertEqual(response.status, 403)
        self.assertIn("error", json.loads(response.read()))
        connection.close()

    def test_invalid_payloads_and_paths_are_json_errors(self):
        for raw in (b"{", b"[]", b"null", b"\xff"):
            self.assertEqual(self.request("POST", "/api/tasks/P0-01", raw=raw)[0], 400)
        for extra in ({"status": "invalid"}, {"evidence": [""]}, {"evidence": None},
                      {"evidence": "text"}, {"due": "2026-99-99"}, {"owner": 7},
                      {"due": None}, {"unknown": "bad"}, {}):
            self.change(extra=extra, expected=400)
        self.change("unknown", {"owner": "测试"}, expected=404)
        for path in ("/../README.md", "/%2e%2e/README.md", "/api/workspace?bad=1"):
            self.assertEqual(self.request("GET", path)[0], 400)
        for path in ("/api/documents/unknown", "/server.py", "/02-research-design/work-items.json"):
            status, result = self.request("GET", path)
            self.assertEqual(status, 404)
            self.assertIn("error", result)
        for method in ("PUT", "DELETE"):
            status, result = self.request(method, "/api/workspace")
            self.assertEqual(status, 501)
            self.assertEqual(result["error"], "不支持此 HTTP 请求方法。")

    def test_framework_header_error_is_chinese_and_keeps_status(self):
        status, result = self.request("GET", "/health", headers={"X-Long": "x" * 65536})
        self.assertEqual(status, 431)
        self.assertEqual(result["error"], "请求头过长或数量过多。")

    def test_cross_site_top_document_navigation_only(self):
        navigation = {"Sec-Fetch-Site": "cross-site", "Sec-Fetch-Mode": "navigate",
                      "Sec-Fetch-Dest": "document", "Sec-Fetch-User": "?1"}
        # 浏览器顶层 GET 导航不发送 Origin；使用 http.client 复现该请求头组合。
        def navigate(path, overrides=None, method="GET", payload=None):
            headers = dict(navigation)
            headers.update(overrides or {})
            connection = http.client.HTTPConnection("127.0.0.1", self.httpd.server_port, timeout=5)
            try:
                connection.request(method, path, body=payload, headers=headers)
                response = connection.getresponse()
                body = response.read()
                return response.status, body
            finally:
                connection.close()

        for path in ("/", "/index.html"):
            status, body = navigate(path)
            self.assertEqual(status, 200, body)
            self.assertIn(b"<html", body.lower())
            for overrides in ({"Sec-Fetch-Mode": "cors"}, {"Sec-Fetch-Dest": "empty"},
                              {"Sec-Fetch-Dest": "iframe"}, {"Sec-Fetch-Mode": ""},
                              {"Host": "evil.example"}, {"Origin": "http://evil.example"}):
                self.assertEqual(navigate(path, overrides)[0], 403)
        for path in ("/api/workspace", "/api/documents/readme", "/health", "/app.js", "/styles.css"):
            self.assertEqual(navigate(path)[0], 403)
        ws = self.workspace()
        payload = {"revision": ws["revision"], "csrf_token": ws["csrf_token"], "owner": "不得写入"}
        status, result = self.request("POST", "/api/tasks/P0-01", payload, headers=navigation)
        self.assertEqual(status, 403, result)
        self.assertNotEqual(self.task(self.workspace())["owner"], "不得写入")

    def test_request_length_and_content_type(self):
        self.assertEqual(self.request("POST", "/api/tasks/P0-01", raw=b"x" * (server.MAX_BODY + 1))[0], 400)
        self.assertEqual(self.request("POST", "/api/tasks/P0-01", raw=b"{}", headers={"Content-Type": "text/plain"})[0], 400)
        self.assertEqual(self.request("POST", "/api/tasks/P0-01", raw=b"{}", headers={"Content-Length": "n/a"})[0], 400)

    def test_render_failure_leaves_originals_untouched(self):
        before = self.originals()
        with patch.object(self.httpd.workspace.progress, "render", side_effect=OSError("模拟文档生成失败")):
            result = self.change(extra={"owner": "应回滚"}, expected=500)
        self.assertIn("保存未完成", result["error"])
        self.assertEqual(before, self.originals())

    def test_partial_save_failure_rolls_back_exact_original_bytes(self):
        before = self.originals()
        actual = server.atomic_bytes
        failed = False

        def fail_once(path, content):
            nonlocal failed
            if path.name == "CHECKLIST.md" and not failed:
                failed = True
                raise OSError("模拟第三文件保存失败")
            actual(path, content)

        with patch.object(server, "atomic_bytes", side_effect=fail_once):
            result = self.change(extra={"owner": "应回滚"}, expected=500)
        self.assertIn("已恢复原始三文件", result["error"])
        self.assertEqual(before, self.originals())

    def test_conflict_during_render_aborts_before_replace(self):
        actual = self.httpd.workspace.progress.render
        ledger = self.httpd.workspace.paths[0]

        def external_write(data):
            actual(data)
            ledger.write_bytes(ledger.read_bytes() + b"\n")

        with patch.object(self.httpd.workspace.progress, "render", side_effect=external_write):
            self.change(extra={"owner": "不应写入"}, expected=409)
        self.assertNotEqual(self.task(self.workspace())["owner"], "不应写入")

    def test_non_tool_document_protection(self):
        path = self.httpd.workspace.paths[2]
        path.write_text("人工文档不能覆盖", encoding="utf-8")
        before = self.originals()
        self.change(extra={"owner": "不应覆盖"}, expected=500)
        self.assertEqual(before, self.originals())

    def test_factory_rejects_remote_binding(self):
        with self.assertRaisesRegex(ValueError, "127.0.0.1"):
            server.make_server(self.paper, host="0.0.0.0")

    def test_missing_frontend_has_clear_json_error(self):
        with patch.object(server, "__file__", str(self.fixture / "absent-web" / "server.py")):
            status, result = self.request("GET", "/")
        self.assertEqual(status, 404)
        self.assertIn("前端文件尚未安装", result["error"])

    def test_incomplete_rollback_blocks_further_saves_and_keeps_backup(self):
        actual = server.atomic_bytes

        def fail_save_and_restore(path, content):
            if path.name == "CHECKLIST.md":
                raise OSError("模拟保存失败")
            if path.name == "PROGRESS.md":
                # 第一次替换成功，恢复时由计数器拒绝。
                counts[path] = counts.get(path, 0) + 1
                if counts[path] > 1:
                    raise OSError("模拟恢复失败")
            actual(path, content)

        counts = {}
        with patch.object(server, "atomic_bytes", side_effect=fail_save_and_restore):
            result = self.change(extra={"owner": "回滚故障隔离"}, expected=500)
        self.assertIn("已禁止继续保存", result["error"])
        self.assertTrue(self.httpd.workspace.write_error)
        self.change(extra={"owner": "禁止继续"}, expected=500)
        self.assertTrue(list((self.fixture / "backups").glob("save-*/work-items.json")))


if __name__ == "__main__":
    unittest.main(verbosity=2)
