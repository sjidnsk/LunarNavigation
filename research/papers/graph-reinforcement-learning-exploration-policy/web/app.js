"use strict";

// 页面只消费真实账本；所有动态文本经转义或 textContent 写入。
const $ = (id) => document.getElementById(id);
const escapeHtml = (value) => String(value ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const icons = {
  overview: '<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>',
  tasks: '<path d="m4 6 1.5 1.5L8 4M11 6h9M4 13h3M11 13h9M4 20h3M11 20h9"/>',
  documents: '<path d="M4 4h6a3 3 0 0 1 3 3v14a4 4 0 0 0-4-2H4zM13 7a3 3 0 0 1 3-3h5v15h-5a4 4 0 0 0-3 2"/>',
  search: '<circle cx="10.5" cy="10.5" r="6.5"/><path d="m16 16 5 5"/>',
  arrow: '<path d="M5 12h14m-5-5 5 5-5 5"/>',
  focus: '<circle cx="12" cy="12" r="8"/><path d="M12 8v4l3 2"/>'
};
const icon = (name) => `<svg viewBox="0 0 24 24" aria-hidden="true">${icons[name] || icons.tasks}</svg>`;
const state = {
  workspace: null, view: "overview", stage: "", status: "", query: "", ready: false, page: 1,
  document: "workplan", taskId: "", baseline: null, editRevision: "", pending: null,
  saving: false, loading: false, workspaceGeneration: 0, documentRequest: 0, lastTrigger: null, lastHash: "", toastTimer: null
};
const pageSize = 12;
const closed = (task) => ["done", "waived"].includes(task.status);
const tasks = () => state.workspace?.ledger.tasks || [];
const taskById = (id) => tasks().find((task) => task.id === id);
const fieldsOf = (task) => ({
  status: task.status, owner: task.owner || "", due: task.due_on || "", note: task.note || "",
  evidence: (task.evidence || []).join("\n")
});
const formFields = () => ({
  status: $("edit-status").value, owner: $("edit-owner").value, due: $("edit-due").value,
  note: $("edit-note").value, evidence: $("edit-evidence").value
});
const dirty = () => Boolean(state.taskId && state.baseline && JSON.stringify(formFields()) !== JSON.stringify(state.baseline));
const hasDraft = () => dirty() || Boolean($("reopen-reason").value.trim());
function message(id, text, success = false) {
  const element = $(id);
  element.textContent = text;
  element.hidden = !text;
  element.classList.toggle("success", success);
}
function toast(text) {
  $("toast").textContent = text;
  $("toast").hidden = false;
  clearTimeout(state.toastTimer);
  state.toastTimer = setTimeout(() => { $("toast").hidden = true; }, 4500);
}
async function api(path, body) {
  let response;
  try {
    response = await fetch(path, {
      method: body ? "POST" : "GET", cache: "no-store", credentials: "same-origin",
      ...(body ? { headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) } : {})
    });
  } catch {
    throw new Error("无法连接本地服务。请检查启动窗口是否仍在运行，然后重试；未保存输入已保留。");
  }
  let result;
  try { result = await response.json(); } catch { throw new Error("服务未返回有效数据，请检查启动窗口。"); }
  if (!response.ok) {
    const error = new Error(result.error || `请求失败（${response.status}）`);
    error.status = response.status;
    throw error;
  }
  return result;
}
function parseHash() {
  const params = new URLSearchParams(location.hash.slice(1));
  return {
    view: ["overview", "tasks", "documents"].includes(params.get("view")) ? params.get("view") : "overview",
    stage: params.get("stage") || "", status: params.get("status") || "", query: params.get("q") || "",
    ready: params.get("ready") === "1", document: params.get("doc") || "workplan", taskId: params.get("task") || ""
  };
}
function writeHash() {
  const params = new URLSearchParams({ view: state.view });
  for (const [key, value] of [["stage", state.stage], ["status", state.status], ["q", state.query], ["ready", state.ready ? "1" : ""], ["doc", state.view === "documents" ? state.document : ""], ["task", state.taskId]]) {
    if (value) params.set(key, value);
  }
  state.lastHash = `#${params}`;
  history.replaceState(null, "", state.lastHash);
}
function statusBadge(task) {
  const safeStatus = ["todo", "doing", "blocked", "review", "done", "waived"].includes(task.status) ? task.status : "todo";
  return `<span class="status ${safeStatus}"><span class="status-dot"></span>${escapeHtml(state.workspace.ledger.status_labels[task.status] || task.status)}</span>`;
}
function renderNavigation() {
  const total = tasks().length;
  $("navigation").innerHTML = [["overview", "研究总览"], ["tasks", "全部任务"], ["documents", "研究章程"]].map(([view, name]) =>
    `<button type="button" class="nav-button ${state.view === view && !state.stage ? "active" : ""}" data-view="${view}" ${state.view === view && !state.stage ? 'aria-current="page"' : ""}>${icon(view)}<span>${name}</span>${view === "tasks" ? `<span class="nav-count">${total}</span>` : ""}</button>`).join("");
  $("stage-count").textContent = `${state.workspace.ledger.stages.length}`;
  $("stages").innerHTML = state.workspace.ledger.stages.map((stage) => {
    const subset = tasks().filter((task) => task.stage === stage.id);
    return `<button type="button" class="nav-button stage-button ${state.stage === stage.id && state.view !== "documents" ? "active" : ""}" data-stage="${escapeHtml(stage.id)}" title="${escapeHtml(stage.title)}"><span class="stage-code">${escapeHtml(stage.id)}</span><span class="stage-title">${escapeHtml(stage.title)}</span><span class="nav-count">${subset.filter((task) => task.status === "done").length}/${subset.length - subset.filter((task) => task.status === "waived").length}</span></button>`;
  }).join("");
}
function renderStats() {
  const all = tasks(), done = all.filter((task) => task.status === "done").length;
  const waived = all.filter((task) => task.status === "waived").length, applicable = all.length - waived;
  const counts = [
    ["已验收完成", `${done}<small>/ ${applicable}</small>`, `适用任务 ${applicable ? (100 * done / applicable).toFixed(1) + "%" : "—"} · 不适用 ${waived} 项`],
    ["正在推进", `${all.filter((task) => task.status === "doing").length}<small>项</small>`, "依据实际研究记录"],
    ["现在可推进", `${state.workspace.ready_ids.length}<small>项</small>`, "前置已关闭的待办 / 待验收"],
    ["待验收 / 受阻", `${all.filter((task) => task.status === "review").length}<small>/ ${all.filter((task) => task.status === "blocked").length}</small>`, "优先核对证据与前置条件"]
  ];
  $("stats").innerHTML = counts.map(([label, value, note]) => `<div class="stat"><div class="stat-label">${label}</div><div class="stat-value">${value}</div><div class="stat-note">${note}</div></div>`).join("");
  $("stats").hidden = state.view === "documents";
}
function renderFocus() {
  const active = tasks().find((task) => task.status === "doing");
  $("focus").hidden = state.view !== "overview" || Boolean(state.stage) || !active;
  if (!active) return;
  const pending = state.workspace.pending_dependencies[active.id] || [];
  $("focus").innerHTML = `<div class="focus-symbol">${icon("focus")}</div><div class="focus-text"><small>当前进行中 · ${escapeHtml(active.id)}</small><h2>${escapeHtml(active.title)}</h2><p>${pending.length ? `仍需补齐前置记录：${escapeHtml(pending.join("、"))}。` : "前置条件已关闭。"} 状态不代表实验完成比例。</p></div><button type="button" class="button" data-task="${escapeHtml(active.id)}">查看记录 ${icon("arrow")}</button>`;
}
function renderHeading() {
  const stage = state.workspace.ledger.stages.find((entry) => entry.id === state.stage);
  const title = state.view === "documents" ? "研究章程与记录" : stage ? stage.title : state.view === "tasks" ? "全部研究任务" : "让研究，一步步可追溯。";
  const description = state.view === "documents" ? "集中阅读工作计划、执行清单和进度记录；任务更新会同步生成进度与清单。" : stage ? stage.scheduling : state.view === "tasks" ? "从现状盘点到投稿归档，按阶段与依赖推进，记录真实的产物和验收证据。" : "图强化学习探索策略 · 以研究问题为起点，以可核对的证据推进每一个阶段。";
  $("page-heading").innerHTML = `<div class="eyebrow">${state.view === "documents" ? "RESEARCH LIBRARY" : stage ? `STAGE ${escapeHtml(stage.id)}` : "GRAPH REINFORCEMENT LEARNING"}</div><h1>${escapeHtml(title)}</h1><p>${escapeHtml(description)}</p>`;
  $("breadcrumb").textContent = state.view === "documents" ? "研究章程" : stage ? stage.id : state.view === "tasks" ? "全部任务" : "总览";
  $("table-title").textContent = stage ? `${stage.id} · 阶段任务` : "研究任务";
}
function filteredTasks() {
  const query = state.query.trim().toLocaleLowerCase();
  return tasks().filter((task) =>
    (!state.stage || task.stage === state.stage) &&
    (!state.status || task.status === state.status) &&
    (!state.ready || state.workspace.ready_ids.includes(task.id)) &&
    (!query || [task.id, task.stage, task.title, task.owner, task.action, task.acceptance, task.output, task.note, ...(task.evidence || [])].join(" ").toLocaleLowerCase().includes(query))
  );
}
function renderTable() {
  $("search").value = state.query;
  $("status-filter").value = state.status;
  $("stage-filter").value = state.stage;
  $("ready-filter").setAttribute("aria-pressed", String(state.ready));
  const filtered = filteredTasks(), pages = Math.max(1, Math.ceil(filtered.length / pageSize));
  state.page = Math.min(state.page, pages);
  $("result-count").textContent = `${filtered.length} / ${tasks().length} 项`;
  $("task-rows").innerHTML = filtered.slice((state.page - 1) * pageSize, state.page * pageSize).map((task) => {
    const pending = state.workspace.pending_dependencies[task.id] || [];
    return `<tr><td class="task-id">${escapeHtml(task.id)}</td><td><button type="button" class="task-title-button" data-task="${escapeHtml(task.id)}">${escapeHtml(task.title)}</button><div class="task-meta">${escapeHtml(task.stage)}${task.kind === "gate" ? " · 阶段关口" : task.can_waive ? " · 条件任务" : " · 研究任务"}</div></td><td>${statusBadge(task)}</td><td class="owner-column">${escapeHtml(task.owner)}</td><td class="date-column muted">${escapeHtml(task.due_on || "待排期")}</td><td class="dependency-column"><span class="${pending.length ? "waiting-text" : "ready-text"}">${pending.length ? `待 ${pending.length} 项` : "已就绪"}</span></td></tr>`;
  }).join("");
  $("empty-state").hidden = filtered.length > 0;
  $("page-number").textContent = `${state.page} / ${pages}`;
  $("previous-page").disabled = state.page <= 1;
  $("next-page").disabled = state.page >= pages;
}
function render() {
  if (!state.workspace) return;
  renderNavigation(); renderHeading(); renderStats(); renderFocus(); renderTable();
  $("task-section").hidden = state.view === "documents";
  $("document-section").hidden = state.view !== "documents";
  $("study-id").textContent = state.workspace.ledger.study_id;
}
function installWorkspace(workspace) {
  state.workspace = workspace;
  $("status-filter").innerHTML = '<option value="">全部状态</option>' + Object.entries(workspace.ledger.status_labels).map(([value, label]) => `<option value="${escapeHtml(value)}">${escapeHtml(label)}</option>`).join("");
  $("stage-filter").innerHTML = '<option value="">全部阶段</option>' + workspace.ledger.stages.map((stage) => `<option value="${escapeHtml(stage.id)}">${escapeHtml(stage.id)} ${escapeHtml(stage.title)}</option>`).join("");
  $("sync-time").textContent = `已同步 ${new Date().toLocaleTimeString("zh-CN", { hour12: false, hour: "2-digit", minute: "2-digit" })}`;
  $("sync-time").title = `账本日期：${workspace.ledger.updated_on}\n论文目录：${workspace.paper_root}`;
  render();
}
async function refreshWorkspace() {
  if (state.loading || state.saving) return;
  const generation = state.workspaceGeneration;
  state.loading = true; $("refresh").disabled = true;
  try {
    const fresh = await api("/api/workspace");
    // 写入开始后，先前发起的读取无论何时返回都不能覆盖保存结果或草稿。
    if (generation !== state.workspaceGeneration) return;
    if (state.taskId && hasDraft()) {
      if (fresh.revision !== state.editRevision || fresh.csrf_token !== state.workspace.csrf_token) showConflict(fresh);
      else toast("账本没有变化；未保存输入已保留。");
    } else {
      installWorkspace(fresh);
      if (state.taskId) populateDetail(taskById(state.taskId));
      if (state.view === "documents") await loadDocument();
      message("global-message", "");
    }
  } catch (error) {
    if (generation === state.workspaceGeneration) message("global-message", error.message);
  }
  finally { state.loading = false; $("refresh").disabled = false; }
}
function canLeave() {
  if (state.saving) return false;
  return !hasDraft() || window.confirm("此任务有未保存的输入。离开将放弃这些输入，确定继续吗？");
}
function closeDetail(check = true) {
  if (check && !canLeave()) return false;
  $("task-dialog").close();
  state.taskId = ""; state.baseline = null; state.pending = null;
  $("reopen-reason").value = "";
  writeHash();
  if (state.lastTrigger?.isConnected) state.lastTrigger.focus();
  return true;
}
async function navigate(changes) {
  if (state.taskId && !closeDetail()) return;
  Object.assign(state, changes, { page: 1 });
  writeHash(); render();
  if (state.view === "documents") await loadDocument();
}
function impactFor(id) {
  const impacted = new Set([id]);
  let grew = true;
  while (grew) {
    grew = false;
    for (const task of tasks()) if (!impacted.has(task.id) && task.depends_on.some((dependency) => impacted.has(dependency))) {
      impacted.add(task.id); grew = true;
    }
  }
  return tasks().filter((task) => task.id !== id && impacted.has(task.id));
}
function populateDetail(task) {
  if (!task) { closeDetail(false); toast("该任务已不存在，请查看最新账本。"); return; }
  state.baseline = fieldsOf(task); state.editRevision = state.workspace.revision; state.pending = null;
  $("detail-id").textContent = `${task.id} / ${task.stage}`;
  $("detail-title").textContent = task.title;
  $("detail-badges").innerHTML = statusBadge(task) + `<span>${task.kind === "gate" ? "阶段验收关口" : task.can_waive ? "条件任务" : "必要任务"}</span>`;
  const pending = state.workspace.pending_dependencies[task.id] || [];
  $("detail-description").innerHTML = `<section class="detail-block"><h3>操作内容</h3><p>${escapeHtml(task.action)}</p></section><section class="detail-block"><h3>验收条件</h3><p>${escapeHtml(task.acceptance)}</p></section><section class="detail-block"><h3>预期产物</h3><p class="output-path">${escapeHtml(task.output)}</p></section><section class="detail-block"><h3>前置依赖 · ${pending.length ? `待关闭 ${pending.length} 项` : "已就绪"}</h3><div class="dependency-list">${task.depends_on.length ? task.depends_on.map((id) => `<button class="button subtle" type="button" data-task="${escapeHtml(id)}">${escapeHtml(id)} · ${escapeHtml(state.workspace.ledger.status_labels[taskById(id)?.status] || "未知")}</button>`).join("") : '<span class="muted">无前置任务</span>'}</div></section><div class="detail-dates"><span>开始：${escapeHtml(task.started_on || "未登记")}</span><span>完成：${escapeHtml(task.completed_on || "未登记")}</span></div>`;
  $("edit-status").innerHTML = Object.entries(state.workspace.ledger.status_labels).map(([value, label]) => `<option value="${escapeHtml(value)}">${escapeHtml(label)}</option>`).join("");
  for (const [key, value] of Object.entries(state.baseline)) $(`edit-${key}`).value = value;
  $("reopen-reason").value = "";
  $("conflict-panel").hidden = true; message("form-message", "");
  $("reopen-section").hidden = !closed(task);
  const impact = impactFor(task.id), needReview = impact.filter(closed);
  $("impact-summary").textContent = `查看下游影响：${impact.length} 项，其中 ${needReview.length} 项将回到待验收`;
  $("impact-list").innerHTML = impact.length ? impact.map((entry) => `<div><strong>${escapeHtml(entry.id)}</strong> ${escapeHtml(entry.title)} — ${closed(entry) ? "改为待验收" : entry.status !== "todo" ? "状态保留，追加复核说明" : "保持待办"}</div>`).join("") : "没有依赖此任务的下游任务。";
  $("task-history").innerHTML = task.history.length ? [...task.history].reverse().map((entry) =>
    `<div class="history-item"><time>${escapeHtml(entry.at)}</time><p>${escapeHtml(state.workspace.ledger.status_labels[entry.from] || entry.from)} → ${escapeHtml(state.workspace.ledger.status_labels[entry.to] || entry.to)}</p><p>${escapeHtml(entry.note)}</p><small>负责人：${escapeHtml(entry.owner || "未登记")} · 日期：${escapeHtml(entry.due_on || "未登记")}</small>${(entry.evidence || []).map((evidence) => `<small>证据：${escapeHtml(evidence)}</small>`).join("")}</div>`
  ).join("") : '<p class="history-empty">暂无变更记录。初始状态依据已登记的研究起点。</p>';
  updateDirty();
}
function openTask(id, trigger) {
  const task = taskById(id);
  if (!task) return;
  if (state.taskId && !canLeave()) return;
  if (!$("task-dialog").open) state.lastTrigger = trigger || document.activeElement;
  state.taskId = id;
  populateDetail(task);
  if (!$("task-dialog").open) $("task-dialog").showModal();
  $("task-dialog").scrollTop = 0;
  $("close-detail").focus();
  writeHash();
}
function updateDirty() {
  $("dirty-state").textContent = dirty() ? "有未保存的更改" : "尚未修改";
  $("save-task").disabled = state.saving || Boolean(state.pending) || !dirty();
  $("reopen-task").disabled = state.saving || Boolean(state.pending);
}
function showConflict(fresh, originalError = "") {
  state.pending = fresh;
  const current = fresh.ledger.tasks.find((task) => task.id === state.taskId);
  $("conflict-panel").hidden = false;
  const draft = formFields(), latest = current ? fieldsOf(current) : null;
  const labels = { status: "状态", owner: "负责人", due: "目标日期", note: "说明", evidence: "证据" };
  $("conflict-comparison").innerHTML = latest ? Object.entries(labels).map(([key, label]) => {
    const display = (value) => key === "status" ? fresh.ledger.status_labels[value] || value : value || "（空）";
    return `<div class="conflict-field"><strong>${label}${draft[key] === latest[key] ? " · 相同" : " · 有差异"}</strong>最新记录：${escapeHtml(display(latest[key]))}\n当前草稿：${escapeHtml(display(draft[key]))}</div>`;
  }).join("") : "此任务已在最新账本中移除，请保留需要的文字后关闭详情。";
  $("adopt-draft").disabled = !latest;
  message("form-message", `${originalError ? originalError + " " : ""}账本或服务会话已更新。未保存输入已保留，请核对最新记录后明确选择。`);
  updateDirty();
}
function resolveConflict(keepDraft) {
  if (!state.pending) return;
  const draft = formFields(), reason = $("reopen-reason").value, fresh = state.pending;
  installWorkspace(fresh);
  populateDetail(taskById(state.taskId));
  if (keepDraft && state.taskId) {
    for (const [key, value] of Object.entries(draft)) $(`edit-${key}`).value = value;
    $("reopen-reason").value = reason;
    message("form-message", "已载入最新账本并保留草稿；请再次核对后保存。", true);
  }
  updateDirty();
}
async function saveTask(event) {
  event.preventDefault();
  if (!state.taskId || state.saving || state.pending || !dirty()) return;
  const draft = formFields();
  const payload = {
    revision: state.editRevision, csrf_token: state.workspace.csrf_token,
    owner: draft.owner, due: draft.due, note: draft.note,
    evidence: draft.evidence.split(/\r?\n/).map((entry) => entry.trim()).filter(Boolean)
  };
  // 未改变状态时不重复提交 status，避免把仅改日期误作一次验收。
  if (draft.status !== state.baseline.status) payload.status = draft.status;
  await submitChange(`/api/tasks/${encodeURIComponent(state.taskId)}`, payload, "研究记录已保存。");
}
async function reopenTask() {
  if (state.saving || state.pending) return;
  if (dirty()) { message("form-message", "请先保存或放弃上方表单的更改，再执行重开；重开原因会保留。"); return; }
  const reason = $("reopen-reason").value.trim();
  if (!reason) { message("form-message", "请填写重开原因。"); $("reopen-reason").focus(); return; }
  const impact = impactFor(state.taskId);
  if (!window.confirm(`确认重开 ${state.taskId}？共影响 ${impact.length} 项下游任务，其中 ${impact.filter(closed).length} 项将回到待验收。`)) return;
  await submitChange(`/api/tasks/${encodeURIComponent(state.taskId)}/reopen`, {
    revision: state.editRevision, csrf_token: state.workspace.csrf_token, reason
  }, "任务已重开，下游复核状态已更新。");
}
async function submitChange(path, payload, success) {
  state.workspaceGeneration++;
  state.saving = true; $("edit-fields").disabled = true; $("reopen-reason").disabled = true;
  $("close-detail").disabled = true; updateDirty(); message("form-message", "");
  try {
    const result = await api(path, payload);
    installWorkspace(result); populateDetail(taskById(state.taskId));
    message("form-message", success, true); toast(success);
    if (state.view === "documents") await loadDocument();
  } catch (error) {
    message("form-message", error.message);
    if (error.status === 409 || error.status === 403) {
      try { showConflict(await api("/api/workspace"), error.message); }
      catch (refreshError) { message("form-message", `${error.message} ${refreshError.message}`); }
    }
  } finally {
    state.saving = false; $("edit-fields").disabled = false; $("reopen-reason").disabled = false;
    $("close-detail").disabled = false; updateDirty();
  }
}

// 有限 Markdown 阅读器：创建 DOM 节点，原文 HTML 始终作为文本显示。
const docNames = { "WORKPLAN.md": "workplan", "PROGRESS.md": "progress", "CHECKLIST.md": "checklist", "README.md": "readme" };
function safeLink(target) {
  if (/^https?:\/\//i.test(target)) {
    try { const url = new URL(target); return ["http:", "https:"].includes(url.protocol) ? { href: url.href, external: true } : null; }
    catch { return null; }
  }
  const path = target.split("#")[0];
  // 只有接口允许的四份文档可以在阅读器内跳转。
  const name = path.split("/").pop();
  if (docNames[name]) return { document: docNames[name] };
  return null;
}
function appendInline(parent, text) {
  const pattern = /(`[^`]+`|\*\*[^*]+\*\*|\[[^\]]+\]\([^)]+\))/g;
  let offset = 0;
  for (const match of text.matchAll(pattern)) {
    parent.append(document.createTextNode(text.slice(offset, match.index)));
    const token = match[0];
    if (token.startsWith("`")) {
      const code = document.createElement("code"); code.textContent = token.slice(1, -1); parent.append(code);
    } else if (token.startsWith("**")) {
      const strong = document.createElement("strong"); strong.textContent = token.slice(2, -2); parent.append(strong);
    } else {
      const parts = /^\[([^\]]+)\]\(([^)]+)\)$/.exec(token), link = safeLink(parts[2]);
      const element = document.createElement(link ? "a" : "span");
      element.textContent = parts[1];
      if (link?.document) {
        element.href = `#view=documents&doc=${encodeURIComponent(link.document)}`;
        element.dataset.document = link.document;
      } else if (link) {
        element.href = link.href; element.target = "_blank"; element.rel = "noopener noreferrer";
      } else {
        element.className = "unavailable-link"; element.title = `文件参考：${parts[2]}（请在论文目录中查看）`;
      }
      parent.append(element);
    }
    offset = match.index + token.length;
  }
  parent.append(document.createTextNode(text.slice(offset)));
}
function markdownToDOM(markdown) {
  const fragment = document.createDocumentFragment(), lines = markdown.replace(/\r\n/g, "\n").split("\n");
  let list = null, paragraph = [], code = null, table = null, fence = null;
  const flush = () => {
    if (paragraph.length) { const p = document.createElement("p"); appendInline(p, paragraph.join("\n")); fragment.append(p); paragraph = []; }
    list = null; table = null;
  };
  const cells = (line) => line.trim().replace(/^\|/, "").replace(/\|$/, "").split(/(?<!\\)\|/).map((part) => part.trim().replace(/\\\|/g, "|"));
  for (let index = 0; index < lines.length; index++) {
    const line = lines[index], fenceMatch = /^\s*(`{3,}|~{3,})/.exec(line);
    if (code) {
      if (fenceMatch && fenceMatch[1][0] === fence[0] && fenceMatch[1].length >= fence.length) { code = null; fence = null; }
      else code.textContent += line + "\n";
      continue;
    }
    if (fenceMatch) {
      flush(); fence = fenceMatch[1]; const pre = document.createElement("pre");
      code = document.createElement("code"); pre.append(code); fragment.append(pre); continue;
    }
    if (!line.trim()) { flush(); continue; }
    if (line.trim().startsWith("<!--") && line.trim().endsWith("-->")) continue;
    const heading = /^(#{1,6})\s+(.+)$/.exec(line);
    if (heading) { flush(); const h = document.createElement(`h${heading[1].length}`); appendInline(h, heading[2]); fragment.append(h); continue; }
    if (/^\s*([-*_])(?:\s*\1){2,}\s*$/.test(line)) { flush(); fragment.append(document.createElement("hr")); continue; }
    if (line.includes("|") && index + 1 < lines.length && /^\s*\|?\s*:?-{3,}:?\s*\|/.test(lines[index + 1])) {
      flush(); const wrap = document.createElement("div"); wrap.className = "markdown-table";
      table = document.createElement("table"); const head = document.createElement("thead"), row = document.createElement("tr");
      for (const value of cells(line)) { const th = document.createElement("th"); th.scope = "col"; appendInline(th, value); row.append(th); }
      head.append(row); table.append(head, document.createElement("tbody")); wrap.append(table); fragment.append(wrap); index++; continue;
    }
    if (table && line.includes("|")) {
      const row = document.createElement("tr");
      for (const value of cells(line)) { const td = document.createElement("td"); appendInline(td, value); row.append(td); }
      table.tBodies[0].append(row); continue;
    }
    table = null;
    const item = /^\s*(?:[-*+]|\d+\.)\s+(.+)$/.exec(line);
    if (item) {
      if (paragraph.length) flush();
      const type = /^\s*\d+\./.test(line) ? "OL" : "UL";
      if (!list || list.tagName !== type) { list = document.createElement(type.toLowerCase()); fragment.append(list); }
      const li = document.createElement("li"); appendInline(li, item[1].replace(/^\[ \]\s*/, "☐ ").replace(/^\[x\]\s*/i, "☑ ")); list.append(li); continue;
    }
    if (/^>\s?/.test(line)) { flush(); const quote = document.createElement("blockquote"); appendInline(quote, line.replace(/^>\s?/, "")); fragment.append(quote); continue; }
    list = null; paragraph.push(line);
  }
  flush(); return fragment;
}
async function loadDocument() {
  if (!state.workspace) return;
  const docs = state.workspace.documents;
  if (!docs.some((entry) => entry.id === state.document)) state.document = docs[0].id;
  writeHash();
  $("document-tabs").innerHTML = docs.map((entry) => `<button type="button" class="button ${entry.id === state.document ? "active" : ""}" data-document="${escapeHtml(entry.id)}" ${entry.id === state.document ? 'aria-current="page"' : ""}>${escapeHtml(entry.title)}</button>`).join("");
  $("document-content").textContent = "正在读取文档…";
  const request = ++state.documentRequest;
  try {
    const result = await api(`/api/documents/${encodeURIComponent(state.document)}`);
    if (request !== state.documentRequest) return;
    $("document-content").replaceChildren(markdownToDOM(result.markdown));
  } catch (error) {
    if (request === state.documentRequest) $("document-content").textContent = error.message + " 请使用右上角刷新重试。";
  }
}
function bindEvents() {
  $("search-icon").innerHTML = icon("search");
  $("refresh").addEventListener("click", refreshWorkspace);
  document.addEventListener("click", (event) => {
    const button = event.target.closest("[data-task], [data-view], [data-stage], [data-document]");
    if (!button || !state.workspace) return;
    if (button.dataset.task) { event.preventDefault(); openTask(button.dataset.task, button); }
    else if (button.dataset.document) { event.preventDefault(); navigate({ view: "documents", document: button.dataset.document, stage: "" }); }
    else if (button.dataset.view) navigate({ view: button.dataset.view, stage: "", status: "", ready: false, query: "" });
    else if (button.dataset.stage) navigate({ view: "tasks", stage: button.dataset.stage, status: "", ready: false, query: "" });
  });
  for (const [id, key] of [["search", "query"], ["status-filter", "status"], ["stage-filter", "stage"]]) {
    $(id).addEventListener(id === "search" ? "input" : "change", (event) => {
      state[key] = event.target.value; state.page = 1; writeHash(); render();
    });
  }
  $("ready-filter").addEventListener("click", () => { state.ready = !state.ready; state.page = 1; writeHash(); renderTable(); });
  $("clear-filters").addEventListener("click", () => navigate({ stage: "", status: "", query: "", ready: false }));
  $("previous-page").addEventListener("click", () => { state.page--; renderTable(); });
  $("next-page").addEventListener("click", () => { state.page++; renderTable(); });
  $("close-detail").addEventListener("click", () => closeDetail());
  $("task-dialog").addEventListener("cancel", (event) => { event.preventDefault(); closeDetail(); });
  $("task-form").addEventListener("input", updateDirty);
  $("task-form").addEventListener("change", updateDirty);
  $("task-form").addEventListener("submit", saveTask);
  $("reopen-task").addEventListener("click", reopenTask);
  $("adopt-draft").addEventListener("click", () => resolveConflict(true));
  $("discard-draft").addEventListener("click", () => resolveConflict(false));
  window.addEventListener("beforeunload", (event) => {
    if (hasDraft() || state.saving) { event.preventDefault(); event.returnValue = ""; }
  });
  window.addEventListener("hashchange", async () => {
    if (!state.workspace) return;
    const route = parseHash();
    if (state.taskId && !canLeave()) { history.replaceState(null, "", state.lastHash); return; }
    closeDetail(false);
    const taskId = route.taskId; route.taskId = "";
    Object.assign(state, route, { page: 1 });
    render(); writeHash();
    if (state.view === "documents") await loadDocument();
    if (taskId) openTask(taskId);
  });
}
async function start() {
  bindEvents();
  const route = parseHash(), initialTask = route.taskId;
  route.taskId = ""; Object.assign(state, route);
  $("page-heading").innerHTML = '<div class="eyebrow">RESEARCH WORKSPACE</div><h1>正在打开研究空间…</h1><p>读取本地账本与研究计划。</p>';
  await refreshWorkspace();
  if (state.workspace) {
    if (initialTask) openTask(initialTask);
    writeHash();
  }
}
start();
