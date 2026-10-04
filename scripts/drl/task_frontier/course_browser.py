"""Local, auto-refreshing browser view of the four persisted curve classes."""
import argparse
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
import json
from pathlib import Path
from course_dashboard import NAMES, safe_read

PAGE='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>四阶段 SAC 训练曲线</title><style>body{font:15px system-ui;background:#f5f7fa;color:#152332;margin:24px}header,main{max-width:1600px;margin:auto}h1{font-size:24px;margin:0 0 8px}#status{color:#475569}main{margin:20px auto;display:grid;grid-template-columns:1fr 1fr;gap:20px}article{background:white;border:1px solid #e2e8f0;border-radius:12px;padding:18px}h2{font-size:18px;margin:0 0 6px}p{color:#64748b;font-size:13px;margin:5px 0 12px}img{width:100%;height:auto;display:block}.stamp{font-size:12px;color:#64748b;margin-top:8px}@media(max-width:900px){main{grid-template-columns:1fr}}</style><header><h1>四阶段 SAC 训练曲线</h1><div id="status">正在读取训练状态…</div><p>区内月表 → 区内洞穴 → 区外月表 → 区外洞穴。每15秒刷新；固定状态和完整评测在训练块结束后新增数据点。</p><a href="/course.csv">下载任务分配统计 CSV</a></header><main>'''
for key,title,desc in [('learning','学习统计','Actor / Critic 损失、温度和策略诊断'),('reward','训练奖励','奖励各项和探索收益'),('fixed','固定状态诊断','相同实测状态中的 Q 与动作选择'),('evaluation','课程评测','按四类任务分别画线；每块当前5场景，后续另加5个旧任务场景')]:
    PAGE+=f'<article><h2>{title}</h2><p>{desc}</p><a target="_blank" href="/{key}.png"><img id="{key}" src="/{key}.png"></a><div class="stamp" id="{key}-time"></div><a href="/{key}.csv">下载 CSV</a></article>'
PAGE+='''</main><script>async function refresh(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw new Error(r.status);const s=await r.json();document.querySelector('#status').textContent=`状态 ${s.status} · 第${s.course_phase+1}/4阶段 ${s.task_name} · ${s.phase} · 转移 ${s.actuals.transitions.toLocaleString()} · 更新 ${s.actuals.updates.toLocaleString()}`;for(const [k,t] of Object.entries(s.modified)){if(t===null){document.getElementById(k+'-time').textContent='等待首次曲线记录';continue}const im=document.getElementById(k);if(im.dataset.stamp!==String(t)){im.src=`/${k}.png?v=${t}`;im.dataset.stamp=String(t)}document.getElementById(k+'-time').textContent='图片生成时间：'+new Date(t*1000).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai'})}}catch(e){document.querySelector('#status').textContent='读取状态失败，稍后重试：'+e.message}}refresh();setInterval(refresh,15000)</script></html>'''


def serve(root,port=0,url_file=None):
    root=Path(root).resolve()
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(root/'curves'),**kwargs)
        def payload(self,value,kind):
            raw=value if isinstance(value,bytes) else value.encode('utf-8')
            self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_GET(self):
            target=self.path.split('?')[0]
            if target=='/':return self.payload(PAGE,'text/html; charset=utf-8')
            if target=='/status':
                m=safe_read(root/'manifest.json',{})
                value=dict(status=m.get('status','准备中'),phase=m.get('phase','等待'),course_phase=m.get('course_phase',0),
                    task_name=NAMES.get(m.get('course_task'),'区内月表'),actuals=m.get('actuals',dict(transitions=0,updates=0)))
                value['modified']={k:(root/'curves'/f'{k}.png').stat().st_mtime if (root/'curves'/f'{k}.png').exists() else None
                    for k in ('learning','reward','fixed','evaluation')}
                return self.payload(json.dumps(value,ensure_ascii=False),'application/json; charset=utf-8')
            return super().do_GET()
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    url=f'http://127.0.0.1:{server.server_port}/'
    if url_file:Path(url_file).write_text(url,encoding='utf-8')
    print(url,flush=True);server.serve_forever()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',required=True,type=Path)
    parser.add_argument('--port',type=int,default=0);parser.add_argument('--url-file',type=Path)
    args=parser.parse_args();serve(args.root,args.port,args.url_file)
