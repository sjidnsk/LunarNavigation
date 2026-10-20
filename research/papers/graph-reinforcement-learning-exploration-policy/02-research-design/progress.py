#!/usr/bin/env python3
"""维护论文任务账本并生成进度与详细清单；只使用Python标准库。"""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
from datetime import date, datetime, timedelta, timezone

BASE = Path(__file__).resolve().parent
LEDGER = BASE / "work-items.json"
MARKER = "<!-- 由 progress.py 从 work-items.json 生成；请通过账本更新。 -->"
TERMINAL = {"done", "waived"}
STATES = {"todo", "doing", "blocked", "review", "done", "waived"}


def now():
    return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="microseconds")


def load():
    return json.loads(LEDGER.read_text(encoding="utf-8-sig"))


def lookup(data):
    return {t["id"]: t for t in data["tasks"]}


def pending(task, items):
    return [key for key in task["depends_on"] if items[key]["status"] not in TERMINAL]


def validate(data):
    errors = []
    tasks = data.get("tasks", [])
    items = lookup(data)
    stages = {s["id"] for s in data["stages"]}
    if len(items) != len(tasks):
        errors.append("任务编号重复")
    for t in tasks:
        key = t["id"]
        if t["stage"] not in stages or t["status"] not in STATES:
            errors.append(f"{key}: 阶段或状态非法")
        for dep in t["depends_on"]:
            if dep not in items:
                errors.append(f"{key}: 不存在的依赖 {dep}")
        if t["status"] in TERMINAL and (not t["evidence"] or not t["note"].strip()):
            errors.append(f"{key}: 完成或不适用必须有证据定位和验收/范围说明")
        if any(not isinstance(ref, str) or not ref.strip() for ref in t["evidence"]):
            errors.append(f"{key}: 证据定位不能为空")
        if t.get("kind") == "gate" and t["status"] == "waived":
            errors.append(f"{key}: 阶段关口不能标为不适用")
        elif t["status"] == "waived" and not t.get("can_waive", False):
            errors.append(f"{key}: 此项为必要任务，不允许标为不适用")
        if t["status"] == "done" and all(d in items for d in t["depends_on"]):
            if pending(t, items):
                errors.append(f"{key}: 已完成任务仍有未关闭依赖")
        if t.get("due_on"):
            try:
                date.fromisoformat(t["due_on"])
            except ValueError:
                errors.append(f"{key}: 日期格式须为YYYY-MM-DD")
    seen, active = set(), set()

    def visit(key):
        if key in active:
            errors.append(f"{key}: 依赖存在环")
            return
        if key in seen:
            return
        active.add(key)
        for dep in items[key]["depends_on"]:
            if dep in items:
                visit(dep)
        active.remove(key)
        seen.add(key)

    for key in items:
        visit(key)
    if errors:
        raise ValueError("\n".join(errors))


def atomic_text(path, text):
    """单个明确目标原子替换；异常时仅清理本次明确临时文件。"""
    fd, filename = tempfile.mkstemp(prefix=".progress-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(filename, path)
    finally:
        if os.path.exists(filename):
            Path(filename).unlink()


def cell(value):
    return str(value).replace("|", "／").replace("\n", " ")


def counts(tasks):
    return {status: sum(t["status"] == status for t in tasks) for status in STATES}


def ready(data):
    items = lookup(data)
    return [t for t in data["tasks"]
            if t["status"] in {"todo", "review"} and not pending(t, items)]


def render(data):
    items, labels = lookup(data), data["status_labels"]
    c = counts(data["tasks"])
    total = len(data["tasks"])
    denominator = total - c["waived"]
    ratio = f"{100*c['done']/denominator:.1f}%" if denominator else "—"
    lines = [MARKER, "# 研究进度总览", "",
             f"账本：work-items.json；最近更新：{data['updated_on']}（Asia/Shanghai）。",
             "",
             f"已验收完成 **{c['done']}/{denominator} 项（{ratio}）**；批准不适用 {c['waived']} 项；"
             f"进行中 {c['doing']} 项；待验收 {c['review']} 项；受阻 {c['blocked']} 项。",
             "",
             "上述比例按适用任务数计算，包含阶段关口；不是工时、实验场次数、科学质量或接收概率。"
             "模型效果不支持预期假设时，完整分析与诚实收缩结论仍可通过相应任务验收。",
             "",
             "## 已知起点", ""]
    for k in ("training", "selection", "research_evidence"):
        lines.append("- " + data["baseline"][k])
    lines += ["", "## 当前进行与阻塞", ""]
    active = [t for t in data["tasks"] if t["status"] in {"doing", "blocked", "review"}]
    if not active:
        lines.append("暂无进行、受阻或待验收任务。")
    for t in active:
        wait = ", ".join(pending(t, items)) or "无"
        lines.append(f"- **{t['id']} {t['title']}**：{labels[t['status']]}；前置待关闭：{wait}。"
                     f" {t['note']}")
    lines += ["", "## 现在可推进", "",
              "以下为前置已关闭的待办/待验收项；可同时推进互不依赖的任务。受阻项需先记录解除原因。", ""]
    available = ready(data)
    for t in available[:10]:
        lines.append(f"- **{t['id']} {t['title']}**：{t['action']}")
    if not available:
        lines.append("- 暂无可直接启动项；先处理进行中、受阻或未完成前置。")
    lines += ["", "## 阶段步骤安排表", "",
              "| 阶段 | 已完成/适用项 | 进行/受阻/待验收 | 放行关口 | 排程 |",
              "| --- | --- | --- | --- | --- |"]
    for s in data["stages"]:
        subset = [t for t in data["tasks"] if t["stage"] == s["id"]]
        sc = counts(subset)
        g = next(t for t in subset if t.get("kind") == "gate")
        lines.append(f"| {s['id']} {s['title']} | {sc['done']}/{len(subset)-sc['waived']} "
                     f"（不适用{sc['waived']}） | {sc['doing']}/{sc['blocked']}/{sc['review']} | "
                     f"{g['id']}：{labels[g['status']]} | {s['scheduling']} |")
    lines += ["", "## 全部任务状态", "",
              "任务动作、产物和验收条件见 [详细工作清单](CHECKLIST.md)。",
              "",
              "| 编号 | 任务 | 状态 | 负责人 | 目标日期 | 前置待关闭 |",
              "| --- | --- | --- | --- | --- | --- |"]
    for t in data["tasks"]:
        row = [t["id"], t["title"], labels[t["status"]], t["owner"],
               t.get("due_on") or "待排期", ", ".join(pending(t, items)) or "—"]
        lines.append("| " + " | ".join(map(cell, row)) + " |")
    lines += ["", "## 最近变更", ""]
    events = [(h["at"], t["id"], h) for t in data["tasks"] for h in t["history"]]
    for at, key, h in sorted(events, key=lambda item: item[0], reverse=True)[:12]:
        lines.append(f"- {at} {key}：{h['from']} → {h['to']}；{h['note']}")
    if not events:
        lines.append("暂无状态变更；初始状态依据已记录的用户说明，其余按待办登记。")
    checklist = [MARKER, "# 详细工作清单", "",
                 "本文件为可阅读、可打印清单；请用 progress.py 更新任务账本后重新生成。"
                 "直接修改本文件中的勾选不会更新账本，重新生成会覆盖手工修改。",
                 "",
                 "完成：产物已形成且验收条件经执行者/作者核对。批准不适用：作者有记录地收缩研究范围，"
                 "不等于任务已执行。仅标为条件任务的项目可申请不适用；其他任务及阶段关口不可豁免。"
                 "证据定位由人核实，工具只检查流程一致性。",
                 ""]
    for s in data["stages"]:
        checklist += [f"## {s['id']} {s['title']}", "", s["scheduling"] + "。", ""]
        for t in [x for x in data["tasks"] if x["stage"] == s["id"]]:
            check = "x" if t["status"] == "done" else " "
            checklist += [
                f"- [{check}] **{t['id']} {t['title']}** — {labels[t['status']]}",
                f"  - 负责人：{t['owner']}；目标日期：{t.get('due_on') or '待排期'}；"
                f"前置：{', '.join(t['depends_on']) or '无'}。",
                f"  - 适用性：{'条件任务；范围收缩须有作者决定和结论影响记录' if t.get('can_waive') else '必要任务；按范围形成实际记录，不能整项跳过'}。",
                f"  - 操作：{t['action']}",
                f"  - 产物：{t['output']}（路径均相对本篇论文根目录；未形成的文件不会预先占位）。",
                f"  - 验收：{t['acceptance']}",
                f"  - 实际证据：{'; '.join(t['evidence']) or '待提供'}。",
                f"  - 备注：{t['note'] or '无'}", ""]
    for path, content in [(BASE / "PROGRESS.md", "\n".join(lines) + "\n"),
                          (BASE / "CHECKLIST.md", "\n".join(checklist) + "\n")]:
        if path.exists() and not path.read_text(encoding="utf-8-sig").startswith(MARKER):
            raise ValueError(f"拒绝覆盖非本工具生成的文件：{path}")
        atomic_text(path, content)


def summary(data):
    c = counts(data["tasks"])
    applicable = len(data["tasks"]) - c["waived"]
    print(f"{data['study_id']}：完成 {c['done']}/{applicable}；不适用 {c['waived']}；"
          f"进行 {c['doing']}；待验收 {c['review']}；受阻 {c['blocked']}")
    print(f"查看 {BASE / 'PROGRESS.md'}")


def event(task, target, note):
    task["history"].append({"at": now(), "from": task["status"], "to": target, "note": note,
                            "owner": task["owner"], "due_on": task.get("due_on"),
                            "evidence": list(task["evidence"])})
    task["status"] = target


def update(data, args):
    items = lookup(data)
    t = items[args.id]
    target = args.status or t["status"]
    if t["status"] in TERMINAL and target not in TERMINAL:
        raise ValueError("重开已关闭任务请使用 reopen；工具会使已关闭的下游任务重新待验收。")
    if target != t["status"] and t["status"] in TERMINAL:
        raise ValueError("改变已关闭任务的处置请先使用 reopen。")
    if target in {"doing", "done"} and pending(t, items):
        if not (target == "doing" and t["status"] == "doing"):
            raise ValueError("前置未关闭：" + ", ".join(pending(t, items)))
    if args.status and target in {"done", "waived", "review", "blocked"} and not args.note:
        raise ValueError("此状态必须提供 --note，写明验收、范围决定或受阻原因。")
    if args.status and target in TERMINAL and not args.evidence:
        raise ValueError("完成/不适用必须提供本次 --evidence 证据定位，工具不替代人工验收。")
    if target == "waived" and t.get("kind") == "gate":
        raise ValueError("阶段关口不能豁免。")
    if target == "waived" and not t.get("can_waive", False):
        raise ValueError("此项为必要任务，不允许标为不适用；请核对产物或先记录正式范围修订。")
    if args.owner:
        t["owner"] = args.owner
    if args.due:
        date.fromisoformat(args.due)
        t["due_on"] = args.due
    if args.evidence:
        for reference in args.evidence:
            if reference not in t["evidence"]:
                t["evidence"].append(reference)
    if args.note:
        t["note"] = args.note
    if target == "doing" and not t.get("started_on"):
        t["started_on"] = now()[:10]
    if args.status and target in TERMINAL:
        t["completed_on"] = now()[:10]
    event(t, target, args.note or "更新负责人或排期")


def reopen(data, args):
    items = lookup(data)
    target = items[args.id]
    if target["status"] not in TERMINAL:
        raise ValueError("reopen用于重开已完成/不适用任务；普通任务请使用update。")
    event(target, "todo", args.reason)
    target["note"] = "重开：" + args.reason
    target["completed_on"] = None
    impacted, changed = {args.id}, True
    while changed:
        changed = False
        for t in data["tasks"]:
            if t["id"] not in impacted and impacted.intersection(t["depends_on"]):
                impacted.add(t["id"])
                changed = True
    for key in impacted - {args.id}:
        t = items[key]
        message = f"上游{args.id}重开，原证据须复核：{args.reason}"
        if t["status"] in TERMINAL:
            event(t, "review", message)
            t["completed_on"] = None
        if t["status"] != "todo":
            t["note"] = message
    print("下游影响范围：" + ", ".join(sorted(impacted - {args.id})))
    print("此操作只更新账本，不修改模型、数据或停止任何运行。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command")
    for name in ("summary", "next", "validate", "render"):
        sub.add_parser(name)
    show = sub.add_parser("show")
    show.add_argument("id")
    change = sub.add_parser("update")
    change.add_argument("id")
    change.add_argument("--status", choices=sorted(STATES))
    change.add_argument("--owner")
    change.add_argument("--due", help="目标日期YYYY-MM-DD")
    change.add_argument("--note")
    change.add_argument("--evidence", action="append", help="实际文件、运行目录、回执或带校验值的记录")
    change.add_argument("--dry-run", action="store_true")
    again = sub.add_parser("reopen")
    again.add_argument("id")
    again.add_argument("--reason", required=True)
    again.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    data = load()
    validate(data)
    command = args.command or "summary"
    if command in {"update", "reopen"}:
        if command == "update":
            if not any((args.status, args.owner, args.due, args.note, args.evidence)):
                raise ValueError("至少提供一个更新字段。")
            update(data, args)
        else:
            reopen(data, args)
        validate(data)
        if args.dry_run:
            print("DRY-RUN：校验通过；账本和文档未写入。")
            return
        data["updated_on"] = now()
        atomic_text(LEDGER, json.dumps(data, ensure_ascii=False, indent=2) + "\n")
        render(data)
        summary(data)
    elif command == "render":
        render(data)
        summary(data)
    elif command == "next":
        for t in ready(data):
            print(f"{t['id']} [{data['status_labels'][t['status']]}] {t['title']}")
        active = [t for t in data["tasks"] if t["status"] == "doing"]
        for t in active:
            print(f"正在进行：{t['id']} {t['title']}；待补前置："
                  f"{', '.join(pending(t, lookup(data))) or '无'}")
    elif command == "show":
        print(json.dumps(lookup(data)[args.id], ensure_ascii=False, indent=2))
    elif command == "validate":
        print(f"账本结构、状态与依赖校验通过：{len(data['tasks'])}项；不代表科学证据已核验。")
    else:
        summary(data)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        sys.exit(2)
