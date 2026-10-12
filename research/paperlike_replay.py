"""Export a local saved-only XY/RGB replay; no Isaac, model, or new rendering.

Open index.html locally. RGB links point to archived source PNGs; the exported
JSON contains exact measured rows and own-observation projected predictions.
"""
import argparse
import json
import os
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.paperlike_analyze import load
from research.records import write_json
from research.hospital_episode import digest


def export(run, output):
    if output.exists() or run in output.parents:
        raise FileExistsError("Fresh derived output outside source required")
    source = load(run)
    output.mkdir(parents=True, exist_ok=False)
    data = {"run": str(run), "human": source["cfg"].get("stationary_human"),
        "goal": source["cfg"]["scene"]["goal"],
        "robot": [[float(row[k]) for k in ["sim_time", "x", "y", "yaw"]] for row in source["rows"]],
        "events": [{"id": e["request_id"], "time": e["observation"]["sim_time"],
            "pose": e["agent_pose_at_observation"], "tick": e["observation"]["tick"],
            "world": w.tolist(), "mode": e["controller_mode"], "target": e["controller_target_vw"],
            "applied": e["applied_vw"], "image": os.path.relpath(run/e["rgb_observation_reference"], output)}
            for e, w in zip(source["events"], source["worlds"])],
        "source_sha256": {str(p): digest(p) for p in run.rglob("*") if p.is_file()}}
    write_json(output/"coordinates.json", data)
    # Escape script-closing characters in any source path/label.
    payload = json.dumps(data, allow_nan=False).replace("<", "\\u003c")
    html = '''<!doctype html><meta charset="utf-8"><title>TIC-VLA saved positive-control replay</title>
<style>body{font:16px system-ui;background:#101828;color:#eee;margin:24px}h1{font-size:22px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}canvas,img{width:100%;background:white;border-radius:8px}button,input{margin:8px}button{padding:8px 14px}#slider{width:60%}pre{white-space:pre-wrap;font-size:14px}</style>
<h1>TIC-VLA positive control — archived data replay</h1>
<p>No model calls, physics re-execution or sensor re-rendering. Blue: measured XY path. Orange: native prediction projected at its own observation. Green: goal.</p>
<button id="prev">Previous C</button><button id="play">Play</button><button id="next">Next C</button><input id="slider" type="range" min="0" value="0"><span id="label"></span>
<div class="grid"><canvas id="map" width="1100" height="800"></canvas><img id="rgb" alt="Original archived RGB used for this observation"></div><pre id="detail"></pre>
<script>const data=PAYLOAD;
const slider=document.getElementById('slider'), canvas=document.getElementById('map'),ctx=canvas.getContext('2d');
slider.max=data.events.length-1;let timer=null;
const pts=data.robot.map(r=>r.slice(1,3)).concat([data.goal.slice(0,2)]);
const xmin=Math.min(...pts.map(p=>p[0]))-2,xmax=Math.max(...pts.map(p=>p[0]))+2,ymin=Math.min(...pts.map(p=>p[1]))-2,ymax=Math.max(...pts.map(p=>p[1]))+2;
const scale=Math.min(950/(xmax-xmin),650/(ymax-ymin));
const xy=p=>[75+(p[0]-xmin)*scale,725-(p[1]-ymin)*scale];
function line(points,color,width){if(!points.length)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();points.map(xy).forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.stroke()}
function circle(p,r,color,fill){ctx.beginPath();ctx.arc(...xy(p),r*scale,0,Math.PI*2);ctx.strokeStyle=color;ctx.fillStyle=color;fill?ctx.fill():ctx.stroke()}
function show(){const e=data.events[+slider.value];document.getElementById('label').textContent='C'+e.id+' / t='+e.time.toFixed(3)+' sim s';document.getElementById('rgb').src=e.image;
ctx.fillStyle='white';ctx.fillRect(0,0,1100,800);ctx.font='16px sans-serif';ctx.fillStyle='#475467';
for(let x=Math.ceil(xmin);x<xmax;x+=2){line([[x,ymin],[x,ymax]],'#eaecf0',1);ctx.fillText(x.toFixed(0),xy([x,ymin])[0],750)}
for(let y=Math.ceil(ymin);y<ymax;y+=2){line([[xmin,y],[xmax,y]],'#eaecf0',1);ctx.fillText(y.toFixed(0),30,xy([xmin,y])[1])}
ctx.fillText('World X [m]; equal XY scale',400,780);ctx.fillText('World Y [m]',20,25);
line(data.robot.map(r=>r.slice(1,3)),'#d0d5dd',2);line(data.robot.slice(0,e.tick+1).map(r=>r.slice(1,3)),'#175cd3',4);line(e.world,'#dc6803',3);circle(e.pose,.15,'#175cd3',true);
line([e.pose.slice(0,2),[e.pose[0]+.6*Math.cos(e.pose[2]),e.pose[1]+.6*Math.sin(e.pose[2])]],'#101828',3);circle(data.goal,1.5,'#079455',false);circle(data.goal,.12,'#079455',true);
if(data.human){circle(data.human.position,.25,'#d92d20',true);circle(data.human.position,.8569825421953869,'#d92d20',false)}
document.getElementById('detail').textContent=JSON.stringify({run:data.run,request_id:e.id,observation_sim_time:e.time,observation_pose:e.pose,mode:e.mode,target_vw:e.target,applied_vw:e.applied,native:'30x2 forward,left metres; no native yaw/time channel'},null,2)}
function move(d){slider.value=Math.max(0,Math.min(+slider.max,+slider.value+d));show()}
slider.oninput=show;document.getElementById('prev').onclick=()=>move(-1);document.getElementById('next').onclick=()=>move(1);
document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;document.getElementById('play').textContent='Play'}else{timer=setInterval(()=>{if(+slider.value===+slider.max){clearInterval(timer);timer=null;document.getElementById('play').textContent='Play'}else move(1)},200);document.getElementById('play').textContent='Pause'}};show();
</script>'''.replace("PAYLOAD", payload)
    (output/"index.html").write_text(html)
    print(output/"index.html")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run-dir", required=True); p.add_argument("--output-dir", required=True)
    a = p.parse_args(); export(Path(a.run_dir).resolve(), Path(a.output_dir).resolve())
