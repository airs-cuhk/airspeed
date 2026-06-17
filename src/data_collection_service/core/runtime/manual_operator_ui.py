"""Manual operator UI — stateless browser control panel for recording."""

from __future__ import annotations

import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import threading
import time as _time
from typing import Any, Callable


class ManualOperatorUI:
    """Stateless HTTP control panel. Reads platform state, renders, sends actions."""

    def __init__(
        self, *, state_machine, control_router, stream_tracker,
        host: str = "127.0.0.1", port: int = 8765,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._state = state_machine
        self._control = control_router
        self._tracker = stream_tracker
        self._host = host
        self._port = port
        self._log = logger or (lambda _: None)
        self._server: HTTPServer | None = None

    @property
    def url(self) -> str:
        return f"http://{self._host}:{self._port}"

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
                    else:
                        self.send_error(404)
                except (BrokenPipeError, ConnectionResetError):
                    pass

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
                return {
                    "lifecycle": snap.lifecycle.value,
                    "active_episode": snap.active_episode_id,
                    "control_mode": ui._control.mode.value,
                    "pending_episode": ui._control.pending_episode,
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
</style></head><body>
<h1>Data Collection Service</h1>
<div id="toast"></div>

<div id="pending-banner" class="pending-banner"><span class="pending-banner-text">Episode recorded. Delete to discard, or start a new episode to keep it.</span><button class="btn-delete" onclick="act('delete')" id="btn-delete-pending">Delete Episode</button></div>

<div class="row">
<div class="col">
<div class="card">
<div class="card-title">Recording State</div>
<div style="display:flex;align-items:center;gap:16px">
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
<button class="btn-toggle" onclick="act('toggle')">Start / Stop</button>
<button class="btn-delete" onclick="act('delete')" id="btn-delete">Delete Episode</button>
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
function render(d){
  let badge=document.getElementById('state-badge');
  let pending=d.pending_episode;
  let lifecycle=d.lifecycle;
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
  let btns=document.querySelectorAll('#actions button');
  btns.forEach(function(b){b.disabled=!isManual;b.style.opacity=isManual?'1':'0.4'});

  // Pending banner
  let banner=document.getElementById('pending-banner');
  let btnDelBanner=document.getElementById('btn-delete-pending');
  let btnDel=document.getElementById('btn-delete');
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

async function act(a){
  try{
    let r=await fetch('/'+a,{method:'POST'});
    let d=await r.json();
    let cls=d.accepted?'toast-msg':'toast-msg toast-err';
    let t=document.getElementById('toast');
    t.innerHTML+='<div class=\"'+cls+'\">'+a+': '+d.message+'</div>';
    setTimeout(function(){let m=t.querySelector('.toast-msg');if(m)m.remove()},4000);
  }catch(e){
    console.error(a+' failed:',e);
    let t=document.getElementById('toast');
    t.innerHTML+='<div class=\"toast-msg toast-err\">'+a+': '+e+'</div>';
  }
}
</script></body></html>"""


__all__ = ["ManualOperatorUI"]
