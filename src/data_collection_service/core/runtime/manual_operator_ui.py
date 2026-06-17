"""Manual operator UI — stateless browser control panel for recording."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from socketserver import ThreadingMixIn
import threading
import time as _time
from typing import Any, Callable


class ManualOperatorUI:
    """Stateless HTTP control panel. Reads platform state, renders, sends actions."""

    def __init__(
        self, *, state_machine, control_router, stream_tracker,
        storage_root: str = "data/episodes",
        on_task_changed: Callable[[str | None], None] | None = None,
        host: str = "127.0.0.1", port: int = 8765,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._state = state_machine
        self._control = control_router
        self._tracker = stream_tracker
        self._storage_root = Path(storage_root)
        self._on_task_changed = on_task_changed or (lambda _: None)
        self._host = host
        self._port = port
        self._log = logger or (lambda _: None)
        self._server: HTTPServer | None = None
        self._active_task: str | None = None
        self._task_target: int = 0

    @property
    def url(self) -> str:
        return f"http://{self._host}:{self._port}"

    # -- task management --

    def _task_dir(self, task_name: str) -> Path:
        return self._storage_root / task_name

    def _task_meta_path(self, task_name: str) -> Path:
        return self._task_dir(task_name) / "task_meta.json"

    def _count_episodes(self, task_name: str) -> int:
        task_dir = self._task_dir(task_name)
        if not task_dir.is_dir():
            return 0
        return len(list(task_dir.glob("*.h5")))

    def _read_task_meta(self, task_name: str) -> dict | None:
        mp = self._task_meta_path(task_name)
        if not mp.exists():
            return None
        try:
            return json.loads(mp.read_text())
        except (json.JSONDecodeError, OSError):
            return None

    def _write_task_meta(self, task_name: str, meta: dict) -> None:
        mp = self._task_meta_path(task_name)
        mp.write_text(json.dumps(meta, indent=2))

    def _list_tasks(self) -> list[dict]:
        tasks = []
        if not self._storage_root.is_dir():
            return tasks
        for d in sorted(self._storage_root.iterdir()):
            if not d.is_dir() or d.name.startswith("."):
                continue
            meta = self._read_task_meta(d.name)
            if meta is None:
                continue
            current = self._count_episodes(d.name)
            tasks.append({
                "name": d.name,
                "current": current,
                "target": meta.get("target_episodes", 0),
            })
        return tasks

    def _select_task(self, task_name: str) -> dict:
        meta = self._read_task_meta(task_name)
        if meta is None:
            raise ValueError(f"task {task_name!r} not found")
        self._active_task = task_name
        self._task_target = meta.get("target_episodes", 0)
        self._on_task_changed(task_name)
        current = self._count_episodes(task_name)
        return {"ok": True, "task_name": task_name, "current": current, "target": self._task_target}

    def _create_task(self, name: str, target: int) -> dict:
        name = name.strip().replace(" ", "-")
        base = name
        counter = 1
        while self._task_dir(name).exists():
            counter += 1
            name = f"{base}_v{counter}"
        task_dir = self._task_dir(name)
        task_dir.mkdir(parents=True, exist_ok=True)
        meta = {
            "task_name": name,
            "target_episodes": target,
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        self._write_task_meta(name, meta)
        return self._select_task(name)

    def start(self) -> None:
        ui = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                try:
                    if self.path == "/":
                        self._serve_page()
                    elif self.path == "/health":
                        self._serve_json(self._health_data())
                    elif self.path == "/stream":
                        self._serve_sse()
                    elif self.path == "/task/list":
                        self._serve_json({"tasks": ui._list_tasks()})
                    else:
                        self.send_error(404)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                try:
                    action = self.path.lstrip("/")
                    if action in ("start", "stop", "save", "abort", "toggle"):
                        result = ui._control.handle_manual_ui_action(action)
                        self._serve_json({"accepted": result.accepted, "message": result.message, "action": result.action})
                    elif action == "delete":
                        result = ui._control.handle_manual_ui_delete()
                        self._serve_json({"accepted": result.accepted, "message": result.message, "action": result.action})
                    elif action in ("task/create", "task/select"):
                        self._handle_task_post(action)
                    else:
                        self.send_error(404)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _handle_task_post(self, action: str):
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length)) if length > 0 else {}
                try:
                    if action == "task/create":
                        result = ui._create_task(str(body.get("name", "")), int(body.get("target", 0)))
                    elif action == "task/select":
                        result = ui._select_task(str(body.get("name", "")))
                    self._serve_json(result)
                except (ValueError, OSError) as exc:
                    self._serve_json({"ok": False, "error": str(exc)})

            def _health_data(self):
                snap = ui._state.health_snapshot
                stream_snap = ui._tracker.snapshot()
                streams = {}
                for name, (status, metrics) in stream_snap.items():
                    streams[name] = {
                        "status": status.value,
                        "received": metrics.messages_received,
                        "valid": metrics.messages_valid,
                        "invalid": metrics.messages_invalid,
                        "rate": metrics.observed_rate_hz,
                        "age_ms": metrics.last_timestamp_age_ms,
                    }
                current = ui._count_episodes(ui._active_task) if ui._active_task else 0
                return {
                    "lifecycle": snap.lifecycle.value,
                    "active_episode": snap.active_episode_id,
                    "control_mode": ui._control.mode.value,
                    "pending_episode": ui._control.pending_episode,
                    "active_task": ui._active_task,
                    "task_current": current,
                    "task_target": ui._task_target,
                    "streams": streams,
                }

            def _serve_sse(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    # SSE push loop — send health snapshot every 200 ms to browser
                    while True:
                        data = json.dumps(self._health_data())
                        self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        _time.sleep(0.2)  # 5 Hz update rate
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _serve_page(self):
                try:
                    data = self._health_data()
                    html = (_UI_HTML
                        .replace("__LIFECYCLE__", data["lifecycle"])
                        .replace("__EPISODE__", data["active_episode"] or "—"))
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(html.encode("utf-8"))
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _serve_json(self, data):
                try:
                    body = json.dumps(data)
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(body.encode("utf-8"))
                except (BrokenPipeError, ConnectionResetError):
                    pass

        class ThreadingReuseServer(ThreadingMixIn, HTTPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._server = ThreadingReuseServer((self._host, self._port), Handler)
        thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        thread.start()
        self._log(f"Manual UI started at {self.url}")

    def stop(self) -> None:
        self._server = None


_UI_HTML = """<!DOCTYPE html>
<html><head><title>Data Collection Service</title>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font:14px/1.5 ui-monospace,SFMono-Regular,SF Mono,Menlo,Consolas,monospace;
     background:#0d1117;color:#c9d1d9;padding:24px;min-height:100vh}
h1{font-size:18px;font-weight:600;margin-bottom:16px;color:#f0f6fc}
.card{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:16px;margin-bottom:16px}
.card-title{font-size:12px;text-transform:uppercase;letter-spacing:.5px;color:#8b949e;margin-bottom:8px}
.row{display:flex;gap:16px;flex-wrap:wrap}
.col{flex:1;min-width:280px}

.state-badge{display:inline-block;padding:4px 12px;border-radius:20px;font-size:13px;font-weight:600}
.state-idle{background:#1f2937;color:#9ca3af}
.state-recording{background:#7f1d1d;color:#fca5a5}
.state-failed{background:#7f1d1d;color:#fca5a5;animation:pulse 1s infinite}
.state-pending{background:#1a3a5c;color:#79c0ff}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}

.rec-dot{display:none;width:28px;height:28px;border-radius:50%;background:#da3633;
         box-shadow:0 0 12px rgba(218,54,51,0.6);flex-shrink:0}
.rec-dot.active{display:inline-block;animation:rec-breathe 1.2s ease-in-out infinite}
@keyframes rec-breathe{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.35;transform:scale(1.25)}}

.episode-id{color:#58a6ff;font-weight:600}
.step-count{color:#f0f6fc;font-size:24px;font-weight:700}
.metric-label{font-size:11px;color:#8b949e;text-transform:uppercase}

button{font:13px ui-monospace,SFMono-Regular,SF Mono,Menlo,Consolas,monospace;
       padding:8px 20px;margin:4px;border:none;border-radius:6px;cursor:pointer;font-weight:600;
       transition:opacity .15s}
button:hover{opacity:.85}button:active{opacity:.7}
.btn-start{background:#238636;color:#fff}
.btn-stop{background:#9e6a03;color:#fff}
.btn-save{background:#1f6feb;color:#fff}
.btn-abort{background:#da3633;color:#fff}
.btn-toggle{background:#238636;color:#fff}
.btn-delete{background:#6e7681;color:#fff}
.btn-delete.visible{background:#da3633;color:#fff}
.pending-banner{background:#1a3a5c;border:1px solid #1f6feb;border-radius:6px;padding:12px 16px;margin-bottom:16px;display:none;align-items:center;justify-content:space-between}
.pending-banner.visible{display:flex}
.pending-banner-text{color:#79c0ff;font-size:13px;font-weight:600}

table{width:100%;border-collapse:collapse}
th{text-align:left;font-size:11px;text-transform:uppercase;letter-spacing:.5px;
   color:#8b949e;padding:8px 12px;border-bottom:1px solid #30363d}
td{padding:6px 12px;border-bottom:1px solid #21262d;font-size:13px}
tr:hover{background:#1c2128}

.status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px}
.st-healthy .status-dot{background:#3fb950}
.st-degraded .status-dot{background:#d29922}
.st-stale .status-dot{background:#db6d28}
.st-absent .status-dot{background:#f85149}
.st-invalid .status-dot{background:#bc8cff}

.mono{font-variant-numeric:tabular-nums}
.num{text-align:right;font-variant-numeric:tabular-nums}
.footer{font-size:11px;color:#484f58;margin-top:16px;text-align:center}
#toast{position:fixed;bottom:20px;right:20px;max-width:400px;z-index:100}
.toast-msg{background:#238636;color:#fff;padding:10px 16px;border-radius:6px;margin-top:6px;
           font-size:13px;animation:fadeOut 4s forwards}
.toast-err{background:#da3633}
@keyframes fadeOut{0%,70%{opacity:1}100%{opacity:0}}

/* Task management */
.task-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
.task-input{background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#c9d1d9;
            padding:7px 12px;font:13px ui-monospace,SFMono-Regular,SF Mono,Menlo,Consolas,monospace;
            outline:none;width:180px}
.task-input:focus{border-color:#1f6feb}
.task-input-sm{width:90px}
.task-select{background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#c9d1d9;
             padding:7px 12px;font:13px ui-monospace,SFMono-Regular,SF Mono,Menlo,Consolas,monospace;
             outline:none;width:100%;max-width:400px;cursor:pointer}
.task-select:focus{border-color:#1f6feb}
.btn-create{background:#1f6feb;color:#fff}
.task-info{display:flex;align-items:center;gap:8px;font-size:13px;margin-top:4px}
.task-info-name{color:#58a6ff;font-weight:600}
.task-info-count{color:#c9d1d9;font-variant-numeric:tabular-nums}
.task-none{color:#8b949e;font-size:13px}
</style></head><body>
<h1>Data Collection Service</h1>
<div id="toast"></div>

<div class="card">
<div class="card-title">Task</div>
<div class="task-row">
  <input class="task-input" id="task-name" placeholder="task name" autocomplete="off">
  <input class="task-input task-input-sm" id="task-target" placeholder="#episodes" value="10" autocomplete="off">
  <button class="btn-create" onclick="createTask()">Create</button>
</div>
<select class="task-select" id="task-select" onchange="selectTask(this.value)">
  <option value="">— select an existing task —</option>
</select>
<div id="task-info" class="task-info task-none">No task selected. Create or select one to start recording.</div>
</div>

<div id="pending-banner" class="pending-banner"><span class="pending-banner-text">Episode recorded. Delete to discard, or start a new episode to keep it.</span><button class="btn-delete" onclick="act('delete')" id="btn-delete-pending">Delete Episode</button></div>

<div class="row">
<div class="col">
<div class="card">
<div class="card-title">Recording State</div>
<div style="display:flex;align-items:center;gap:16px">
<span class="rec-dot" id="rec-dot"></span>
<span class="state-badge state-__LIFECYCLE__" id="state-badge">__LIFECYCLE__</span>
<div><div class="metric-label">Episode</div><div class="episode-id" id="ep-id">__EPISODE__</div></div>
<div><div class="metric-label">Control</div><div style="color:#8b949e;font-size:13px" id="ctrl-mode">—</div></div>
</div>
</div>
</div>
<div class="col">
<div class="card">
<div class="card-title">Controls</div>
<div id="actions">
<button class="btn-toggle" id="btn-toggle" onclick="act('toggle')" disabled>Start / Stop</button>
<button class="btn-delete" onclick="act('delete')" id="btn-delete" disabled>Delete Episode</button>
<button class="btn-abort" onclick="act('abort')">Abort</button>
</div>
</div>
</div>
</div>

<div class="card">
<div class="card-title">Stream Status</div>
<div id="streams"><span style="color:#8b949e">Loading...</span></div>
</div>

<div class="footer">Data Collection Service · single source of truth · real-time</div>

<script>
var _activeTask=null;
var _recording=false;
var _tasks=[];

function render(d){
  let badge=document.getElementById('state-badge');
  let pending=d.pending_episode;
  let lifecycle=d.lifecycle;
  _recording=(lifecycle==='recording');
  _activeTask=d.active_task||null;

  let dot=document.getElementById('rec-dot');
  if(lifecycle==='recording'){
    dot.classList.add('active');
  }else{
    dot.classList.remove('active');
  }

  if(pending){
    badge.textContent='pending';
    badge.className='state-badge state-pending';
  }else{
    badge.textContent=lifecycle;
    badge.className='state-badge state-'+lifecycle;
  }
  document.getElementById('ep-id').textContent=d.active_episode||'—';
  let mode=d.control_mode||'';
  document.getElementById('ctrl-mode').textContent=mode;
  let isManual=mode==='manual_ui';
  let hasTask=!!_activeTask;

  // Controls
  let btnToggle=document.getElementById('btn-toggle');
  let btnDel=document.getElementById('btn-delete');
  let btnAbort=document.querySelector('.btn-abort');
  let allBtns=[btnToggle,btnDel,btnAbort];
  allBtns.forEach(function(b){
    b.disabled=!isManual;
    b.style.opacity=isManual?'1':'0.4';
  });
  // Toggle requires task
  if(isManual){
    btnToggle.disabled=!hasTask;
    btnToggle.textContent=_recording?'Stop Recording':'Start Recording';
    if(!hasTask){btnToggle.style.opacity='0.4';}
  }

  // Pending banner
  let banner=document.getElementById('pending-banner');
  let btnDelBanner=document.getElementById('btn-delete-pending');
  if(pending && isManual){
    banner.classList.add('visible');
    btnDelBanner.disabled=false; btnDelBanner.style.opacity='1';
    btnDel.classList.add('visible');
    btnDel.disabled=false; btnDel.style.opacity='1';
  }else{
    banner.classList.remove('visible');
    btnDelBanner.disabled=true; btnDelBanner.style.opacity='0.4';
    btnDel.classList.remove('visible');
    if(!pending){btnDel.disabled=true;btnDel.style.opacity='0.4';}
  }

  // Task info display
  let ti=document.getElementById('task-info');
  if(hasTask){
    let cur=d.task_current||0;
    let tgt=d.task_target||0;
    ti.innerHTML='Active: <span class="task-info-name">'+_activeTask+'</span> &middot; <span class="task-info-count">'+cur+' / '+tgt+'</span>';
    ti.className='task-info';
  }else if(!_recording && !pending){
    ti.textContent='No task selected. Create or select one to start recording.';
    ti.className='task-info task-none';
  }

  // Disable dropdown + create while recording
  document.getElementById('task-select').disabled=_recording;
  document.getElementById('task-name').disabled=_recording;
  document.getElementById('task-target').disabled=_recording;

  let streams=d.streams||{};
  if(Object.keys(streams).length===0){
    document.getElementById('streams').innerHTML='<span style="color:#8b949e">No streams configured</span>';
    return;
  }
  let h='<table><thead><tr><th>Stream</th><th>Status</th><th class="num">Recv</th><th class="num">Valid</th><th class="num">Invalid</th><th class="num">Rate Hz</th><th>Age ms</th></tr></thead><tbody>';
  for(let[name, s] of Object.entries(streams)){
    let cls='st-'+s.status;
    h+='<tr class="'+cls+'"><td>'+name+'</td>';
    h+='<td><span class="status-dot"></span>'+s.status+'</td>';
    h+='<td class="num">'+(s.received||0)+'</td>';
    h+='<td class="num">'+(s.valid||0)+'</td>';
    h+='<td class="num">'+(s.invalid||0)+'</td>';
    h+='<td class="num">'+(s.rate!=null?s.rate.toFixed(1):'—')+'</td>';
    h+='<td class="num">'+(s.age_ms!=null?s.age_ms.toFixed(0):'—')+'</td>';
    h+='</tr>';
  }
  h+='</tbody></table>';
  document.getElementById('streams').innerHTML=h;
}

var es=new EventSource('/stream');
es.onmessage=function(e){render(JSON.parse(e.data))};

var _taskListCache='';

async function refreshTasks(){
  if(_recording) return;
  try{
    let r=await fetch('/task/list');
    let d=await r.json();
    _tasks=d.tasks||[];
    // Only rebuild DOM if the task list actually changed
    let key=_tasks.map(function(t){return t.name+'/'+t.current+'/'+t.target;}).join(',');
    if(key===_taskListCache) return;
    _taskListCache=key;
    let sel=document.getElementById('task-select');
    let cur=sel.value;
    sel.innerHTML='<option value="">— select an existing task —</option>';
    _tasks.forEach(function(t){
      let label=t.name+' ('+t.current+'/'+t.target+')';
      sel.innerHTML+='<option value="'+t.name+'">'+label+'</option>';
    });
    if(cur) sel.value=cur;
  }catch(e){console.error('task list failed:',e);}
}

async function createTask(){
  let name=document.getElementById('task-name').value.trim();
  let target=parseInt(document.getElementById('task-target').value)||10;
  if(!name){toastMsg('task/create','Enter a task name',false);return;}
  try{
    let r=await fetch('/task/create',{method:'POST',body:JSON.stringify({name:name,target:target})});
    let d=await r.json();
    if(d.ok){
      _activeTask=d.task_name;
      document.getElementById('task-name').value='';
      document.getElementById('task-select').value=d.task_name;
      toastMsg('task/create','Created: '+d.task_name,true);
    }else{
      toastMsg('task/create',d.error||'failed',false);
    }
  }catch(e){toastMsg('task/create',String(e),false);}
}

async function selectTask(name){
  if(!name){_activeTask=null;return;}
  try{
    let r=await fetch('/task/select',{method:'POST',body:JSON.stringify({name:name})});
    let d=await r.json();
    if(d.ok){
      _activeTask=d.task_name;
      toastMsg('task/select','Selected: '+d.task_name+' ('+d.current+'/'+d.target+')',true);
    }
  }catch(e){console.error(e);}
}

async function act(a){
  try{
    let r=await fetch('/'+a,{method:'POST'});
    let d=await r.json();
    let ok=d.accepted!==undefined?d.accepted:d.ok;
    let cls=ok?'toast-msg':'toast-msg toast-err';
    let msg=d.message||d.error||'';
    if(d.action==='start'||d.action==='stop'){msg=(d.action==='start'?'Started':'Stopped')+' recording';}
    let t=document.getElementById('toast');
    t.innerHTML+='<div class=\"'+cls+'\">'+a+': '+msg+'</div>';
    setTimeout(function(){let m=t.querySelector('.toast-msg');if(m)m.remove()},4000);
  }catch(e){
    console.error(a+' failed:',e);
    let t=document.getElementById('toast');
    t.innerHTML+='<div class=\"toast-msg toast-err\">'+a+': '+e+'</div>';
  }
}

function toastMsg(action,msg,ok){
  let cls=ok?'toast-msg':'toast-msg toast-err';
  let t=document.getElementById('toast');
  t.innerHTML+='<div class=\"'+cls+'\">'+action+': '+msg+'</div>';
  setTimeout(function(){let m=t.querySelector('.toast-msg');if(m)m.remove()},4000);
}

refreshTasks();
setInterval(refreshTasks, 2000);
</script></body></html>"""


__all__ = ["ManualOperatorUI"]
