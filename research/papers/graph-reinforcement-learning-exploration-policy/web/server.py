#!/usr/bin/env python3
"""论文研究工作台：仅本机使用的标准库 HTTP 适配层。"""
import argparse
from contextlib import redirect_stdout
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import importlib.util
import io
import json
import os
from pathlib import Path
import secrets
import sys
import tempfile
from types import SimpleNamespace
from urllib.parse import unquote, urlsplit
import webbrowser

sys.dont_write_bytecode = True
DEFAULT_ARTIFACTS = Path(__file__).resolve().parent / ".local"
MAX_BODY = 256 * 1024
STATIC_FILES = {"/": "index.html", "/index.html": "index.html",
                "/styles.css": "styles.css", "/app.js": "app.js"}
DOCUMENTS = {"workplan": ("研究工作计划", "02-research-design/WORKPLAN.md"),
             "progress": ("研究进度", "02-research-design/PROGRESS.md"),
             "checklist": ("详细工作清单", "02-research-design/CHECKLIST.md"),
             "readme": ("论文研究说明", "README.md")}


class RequestError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def atomic_bytes(path, content):
    """精确恢复原始字节；临时文件只生成于目标所在 D 盘目录。"""
    fd, filename = tempfile.mkstemp(prefix=".dashboard-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(filename, path)
    finally:
        if os.path.exists(filename):
            Path(filename).unlink()


class Workspace:
    def __init__(self, paper_root, backup_dir):
        self.root = Path(paper_root).resolve(strict=True)
        self.base = self.root / "02-research-design"
        self.paths = [self.base / name for name in
                      ("work-items.json", "PROGRESS.md", "CHECKLIST.md")]
        self.backup_dir = Path(backup_dir).resolve()
        if os.name == "nt" and self.backup_dir.drive.lower() != "d:":
            raise ValueError("备份目录必须明确指定在 D 盘。")
        spec = importlib.util.spec_from_file_location("dashboard_progress", self.base / "progress.py")
        self.progress = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.progress)
        self.token = secrets.token_urlsafe(32)
        self.write_error = None

    def load(self):
        raw = self.paths[0].read_bytes()
        data = json.loads(raw.decode("utf-8-sig"))
        self.progress.validate(data)
        return data, hashlib.sha256(raw).hexdigest()

    def snapshot(self, data=None, revision=None):
        if data is None:
            data, revision = self.load()
        items = self.progress.lookup(data)
        return {"revision": revision, "csrf_token": self.token, "ledger": data,
                "ready_ids": [task["id"] for task in self.progress.ready(data)],
                "pending_dependencies": {task["id"]: self.progress.pending(task, items)
                                         for task in data["tasks"]},
                "paper_root": str(self.root),
                "documents": [{"id": key, "title": title}
                              for key, (title, _) in DOCUMENTS.items()]}

    def change(self, key, payload, reopen=False):
        if self.write_error:
            raise RuntimeError(self.write_error)
        data, revision = self.load()
        if payload.get("revision") != revision:
            raise RequestError(409, "账本已被其他页面或命令行更新，请刷新后重新编辑。")
        items = self.progress.lookup(data)
        if key not in items:
            raise RequestError(404, "任务不存在。")
        updated = copy.deepcopy(data)
        task = self.progress.lookup(updated)[key]
        allowed = {"revision", "csrf_token", "reason"} if reopen else {
            "revision", "csrf_token", "status", "owner", "due", "note", "evidence"}
        if set(payload) - allowed:
            raise RequestError(400, "包含不支持的更新字段。")
        for field in allowed - {"evidence"}:
            if field in payload and not isinstance(payload[field], str):
                raise RequestError(400, f"{field} 必须为文本。")
        if reopen:
            reason = payload.get("reason", "").strip()
            if not reason:
                raise RequestError(400, "重开必须填写原因。")
            with redirect_stdout(io.StringIO()):
                self.progress.reopen(updated, SimpleNamespace(id=key, reason=reason))
        else:
            if not set(payload).intersection({"status", "owner", "due", "note", "evidence"}):
                raise RequestError(400, "至少提供一个更新字段。")
            if "status" in payload and payload["status"] not in self.progress.STATES:
                raise RequestError(400, "任务状态非法。")
            evidence = payload.get("evidence")
            if evidence is not None and (not isinstance(evidence, list) or
                    any(not isinstance(ref, str) or not ref.strip() for ref in evidence)):
                raise RequestError(400, "证据必须为非空文本组成的列表。")
            if "evidence" in payload and evidence is None:
                raise RequestError(400, "证据必须为列表。")
            # update 的命令行语义是追加证据；网页提交完整编辑值。
            if "evidence" in payload:
                task["evidence"] = list(evidence)
            if "due" in payload and payload["due"] == "":
                task["due_on"] = None
            for field in ("owner", "note"):
                if field in payload:
                    task[field] = payload[field]
            self.progress.update(updated, SimpleNamespace(
                id=key, status=payload.get("status"), owner=payload.get("owner"),
                due=payload.get("due"), note=payload.get("note"), evidence=evidence))
        self.progress.validate(updated)
        updated["updated_on"] = self.progress.now()
        try:
            self.save(updated, revision)
        except RequestError:
            raise
        except Exception as exc:
            raise RuntimeError(f"保存未完成：{exc}") from exc
        return self.snapshot()

    def save(self, data, expected_revision):
        """预生成三文件，保留完整备份；失败恢复已替换的明确文件。"""
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        backup = Path(tempfile.mkdtemp(prefix="save-", dir=self.backup_dir))
        originals = {path: path.read_bytes() if path.exists() else None for path in self.paths}
        for path, content in originals.items():
            if content is not None:
                (backup / path.name).write_bytes(content)
        staged = backup / "staged"
        staged.mkdir()
        for path in self.paths[1:]:
            if originals[path] is not None:
                (staged / path.name).write_bytes(originals[path])
        old_base = self.progress.BASE
        try:
            self.progress.BASE = staged
            self.progress.render(data)
        finally:
            self.progress.BASE = old_base
        contents = {self.paths[0]: (json.dumps(data, ensure_ascii=False, indent=2) + "\n").encode("utf-8")}
        contents.update({path: (staged / path.name).read_bytes() for path in self.paths[1:]})
        if self.load()[1] != expected_revision:
            raise RequestError(409, "账本已被外部工具更新，请刷新后重新编辑。")
        replaced = []
        try:
            for path, content in contents.items():
                atomic_bytes(path, content)
                replaced.append(path)
        except Exception as exc:
            failures = []
            for path in reversed(replaced):
                try:
                    if originals[path] is None:
                        path.unlink(missing_ok=True)
                    else:
                        atomic_bytes(path, originals[path])
                except Exception as restore_error:
                    failures.append(f"{path.name}: {restore_error}")
            if failures:
                self.write_error = f"保存失败且恢复不完整，已禁止继续保存；请停止服务并从 {backup} 恢复：" + "; ".join(failures)
                raise RuntimeError(self.write_error) from exc
            raise RuntimeError(f"保存失败，已恢复原始三文件；备份：{backup}") from exc


class DashboardServer(HTTPServer):
    # HTTPServer 串行执行请求；网页与外部 CLI 仍须遵守单写入者约定。
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    server_version = "ResearchDashboard/1.0"

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def log_message(self, format, *args):
        pass

    def send_json(self, status, value):
        body = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def send_error(self, code, message=None, explain=None):
        self.close_connection = True
        # 框架提供的 message 为英文；统一给出中文并保留原 HTTP 状态。
        messages = {400: "HTTP 请求格式无效。", 404: "请求的资源不存在。",
                    414: "请求地址过长。", 431: "请求头过长或数量过多。",
                    501: "不支持此 HTTP 请求方法。", 505: "不支持此 HTTP 协议版本。"}
        self.send_json(code, {"error": messages.get(code, f"HTTP 请求失败，状态码：{code}。")})

    def check_source(self, write=False):
        host = self.headers.get_all("Host", [])
        expected = f"127.0.0.1:{self.server.server_port}"
        if host != [expected]:
            raise RequestError(403, "Host 不受信任，请使用服务器显示的本机地址。")
        origins = self.headers.get_all("Origin", [])
        if (write or origins) and origins != [f"http://{expected}"]:
            raise RequestError(403, "请求必须来自当前工作台同源页面。")
        # 允许从外站链接安全打开首页；只读 API 和资源 fetch 不适用此例外。
        document_navigation = (
            not write and self.command == "GET" and self.path in {"/", "/index.html"}
            and self.headers.get_all("Sec-Fetch-Mode", []) == ["navigate"]
            and self.headers.get_all("Sec-Fetch-Dest", []) == ["document"])
        if self.headers.get("Sec-Fetch-Site") == "cross-site" and not document_navigation:
            raise RequestError(403, "拒绝跨站请求。")

    def do_GET(self):
        self.dispatch(False)

    def do_POST(self):
        self.dispatch(True)

    def dispatch(self, write):
        try:
            self.check_source(write)
            parsed = urlsplit(self.path)
            path = unquote(parsed.path)
            if parsed.query or parsed.fragment or parsed.scheme or parsed.netloc or "\\" in path or ".." in path.split("/"):
                raise RequestError(400, "请求路径非法。")
            workspace = self.server.workspace
            if write:
                payload = self.read_payload()
                if not isinstance(payload.get("csrf_token"), str) or not secrets.compare_digest(
                        payload["csrf_token"].encode("utf-8", errors="replace"), workspace.token.encode("ascii")):
                    raise RequestError(403, "会话令牌无效，请刷新页面。")
                parts = path.strip("/").split("/")
                if len(parts) not in {3, 4} or parts[:2] != ["api", "tasks"] or (len(parts) == 4 and parts[3] != "reopen"):
                    raise RequestError(404, "接口不存在。")
                return self.send_json(200, workspace.change(parts[2], payload, reopen=len(parts) == 4))
            if path == "/health":
                return self.send_json(200, {"status": "ok", "study_id": workspace.load()[0]["study_id"]})
            if path == "/api/workspace":
                return self.send_json(200, workspace.snapshot())
            if path.startswith("/api/documents/"):
                key = path[len("/api/documents/"):]
                if key not in DOCUMENTS:
                    raise RequestError(404, "文档不在可读清单内。")
                title, relative = DOCUMENTS[key]
                target = (workspace.root / relative).resolve()
                if not target.is_relative_to(workspace.root):
                    raise RequestError(403, "文档路径超出论文目录。")
                if not target.is_file():
                    raise RequestError(404, "文档尚未创建。")
                return self.send_json(200, {"id": key, "title": title, "markdown": target.read_text(encoding="utf-8-sig")})
            if path not in STATIC_FILES:
                raise RequestError(404, "页面或接口不存在。")
            static_root = Path(__file__).resolve().parent
            target = (static_root / STATIC_FILES[path]).resolve()
            if not target.is_relative_to(static_root) or not target.is_file():
                raise RequestError(404, "前端文件尚未安装，请先准备工作台 web 页面。")
            content = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[target.suffix] + "; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(content)
        except RequestError as exc:
            self.close_connection = True
            self.send_json(exc.status, {"error": str(exc)})
        except (ValueError, KeyError) as exc:
            self.send_json(400, {"error": f"校验未通过：{exc}"})
        except Exception as exc:
            self.send_json(500, {"error": f"读取或保存失败：{exc}"})

    def read_payload(self):
        if self.headers.get("Transfer-Encoding"):
            raise RequestError(400, "不支持分块请求。")
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1 or not lengths[0].isdigit():
            raise RequestError(400, "必须提供有效的请求长度。")
        length = int(lengths[0])
        if not 0 < length <= MAX_BODY:
            raise RequestError(400, "请求内容为空或超过 256 KiB 上限。")
        if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
            raise RequestError(400, "请求内容必须使用 application/json。")
        raw = self.rfile.read(length)
        if len(raw) != length:
            raise RequestError(400, "请求内容长度不完整。")
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeError) as exc:
            raise RequestError(400, "请求不是有效的 UTF-8 JSON。") from exc
        if not isinstance(payload, dict):
            raise RequestError(400, "请求 JSON 必须为对象。")
        return payload


def make_server(paper_root, host="127.0.0.1", port=0, backup_dir=None):
    """创建后由调用方 serve_forever；测试可用 thread 启动并 shutdown。"""
    if host != "127.0.0.1":
        raise ValueError("工作台只允许监听 127.0.0.1。")
    workspace = Workspace(paper_root, backup_dir or DEFAULT_ARTIFACTS / "backups")
    workspace.load()  # 创建服务时只读核对，不自动刷新文档。
    server = DashboardServer((host, port), Handler)
    server.workspace = workspace
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paper-root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--backup-dir", type=Path, default=DEFAULT_ARTIFACTS / "backups")
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()
    with make_server(args.paper_root, port=args.port, backup_dir=args.backup_dir) as server:
        url = f"http://127.0.0.1:{server.server_port}/"
        print(f"研究工作台：{url}\n论文目录：{server.workspace.root}\n备份目录：{server.workspace.backup_dir}", flush=True)
        if args.open:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
