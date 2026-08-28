"""Browser-based camera-coordinate teach and pick/place controller."""

from __future__ import annotations

import json
import io
import math
import os
import signal
import threading
import time
import zipfile
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray
from .dofbot_kinematics import forward_kinematics_servo, inverse_kinematics_servo


CALIBRATION_FILE = Path('/home/intelions/ros2_ws/config/camera_pick_calibration.json')
DATASET_DIR = Path('/home/intelions/ros2_ws/datasets/handle_segmentation/raw')
PLANAR_CALIBRATION_FILE = Path('/home/intelions/ros2_ws/config/planar_xy_calibration.json')
RGB_TOPIC = '/ascamera/ascamera_node/rgb0/image'
DEPTH_TOPIC = '/ascamera/ascamera_node/depth0/image_raw'
CAMERA_INFO_TOPIC = '/ascamera/ascamera_node/rgb0/camera_info'

PAGE = r'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DOFBOT 카메라 좌표 제어</title>
<style>
:root{font-family:-apple-system,BlinkMacSystemFont,"Noto Sans KR",sans-serif;color:#172033;background:#eef2f7}
*{box-sizing:border-box}body{margin:0;padding:18px}.wrap{max-width:1180px;margin:auto;display:grid;grid-template-columns:minmax(480px,2fr) minmax(330px,1fr);gap:16px}
.card{background:#fff;border-radius:14px;box-shadow:0 3px 14px #17203318;padding:14px}h1{font-size:22px;margin:0 0 6px}.muted{color:#657086;font-size:13px}.cameras{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}.camera-wrap:nth-child(1){order:2}.camera-wrap:nth-child(2){order:1}.camera-wrap h2{font-size:15px;margin:0 0 6px}.camera{position:relative;background:#222;min-height:220px;border-radius:9px;overflow:hidden}.camera img{display:block;width:100%;height:auto;cursor:crosshair}.camera:after{content:"영상을 기다리는 중…";color:#aaa;position:absolute;inset:45% 0;text-align:center;z-index:0}.camera img{position:relative;z-index:1}
fieldset{border:1px solid #dbe2ec;border-radius:10px;margin:12px 0;padding:10px}legend{font-weight:700;padding:0 5px}button{border:0;border-radius:8px;padding:9px 11px;margin:3px;background:#e8edf5;color:#172033;font-weight:650;cursor:pointer}button.primary{background:#2563eb;color:white}button.danger{background:#dc2626;color:white}button:active{transform:translateY(1px)}label{display:flex;align-items:center;justify-content:space-between;margin:6px 3px}input{width:82px;padding:6px;border:1px solid #ccd5e2;border-radius:6px}.line{padding:5px 2px}.status{background:#f1f5f9;border-radius:8px;padding:10px;min-height:44px}.ok{color:#087f5b}.warn{color:#b45309}@media(max-width:850px){.wrap{grid-template-columns:1fr}.cameras{grid-template-columns:1fr}.camera{min-height:240px}}
</style></head><body><div style="max-width:1180px;margin:0 auto 10px"><a href="/xyz">평면 XY + 고정 Z GUI</a> · <a href="/handle">RGB-D 검은 손잡이 필터</a></div><div class="wrap">
<section class="card"><h1>DOFBOT IK 캘리브레이션</h1><div class="muted">두 화면 모두 클릭해 각 카메라의 기준점을 선택할 수 있습니다.</div><div class="cameras"><div class="camera-wrap"><h2>카메라 1 · RGB-D (클릭/보정)</h2><div class="camera primary"><img id="cam" src="/stream" alt="RGB-D 카메라"></div><div class="line" id="cameraState">RGB-D: 확인 중…</div><button onclick="reconnectCamera()">↻ RGB-D 다시 연결</button></div><div class="camera-wrap"><h2>카메라 2 · C270 (클릭/보조)</h2><div class="camera"><img id="cam2" src="/stream/usb" alt="C270 카메라"></div><div class="line" id="camera2State">C270: 확인 중…</div><button onclick="reconnectUsbCamera()">↻ C270 다시 연결</button></div></div><div class="line" id="selected">RGB-D 선택: —</div><div class="line" id="selected2">C270 선택: —</div><div class="muted">픽/드롭과 Depth는 RGB-D 선택을 사용하며 C270 좌표는 두 카메라 보정용으로 분리됩니다.</div></section>
<aside class="card">
<fieldset><legend>학습 이미지 수집</legend><button class="primary" onclick="capturePair()">📷 두 카메라 촬영 · PC에 저장</button><div class="line" id="captures">촬영 다운로드 0세트</div><div class="muted">원본 RGB-D RGB, Depth PNG, C270 RGB, 메타데이터를 ZIP으로 브라우저의 다운로드 폴더에 저장합니다.</div></fieldset>
<fieldset><legend>현재 팔 관절값</legend><div class="line" id="joints">J1 — · J2 — · J3 — · J4 — · J5 — · J6 —</div><div class="muted">손으로 자세를 잡은 뒤 여섯 값이 모두 표시돼야 기록할 수 있습니다.</div></fieldset>
<fieldset><legend>관절 토크</legend><button onclick="torque(false)">Torque OFF</button><button class="primary" onclick="torque(true)">Torque ON</button><div class="muted">OFF: 손으로 팔 이동 · ON: 현재 자세 유지</div></fieldset>
<fieldset><legend>실행 목표 지정</legend><button onclick="target('pick')">현재 클릭을 픽 위치로 지정</button><button onclick="target('drop')">현재 클릭을 드롭 위치로 지정</button><div class="line" id="pick">픽 위치: —</div><div class="line" id="drop">드롭 위치: —</div><div class="muted">안전영역 보정 없이 클릭한 좌표를 목표로만 저장합니다.</div></fieldset>
<fieldset><legend>IK 캘리브레이션 기록</legend><div class="muted">영상에서 팔 끝점 또는 기준점을 클릭한 뒤 현재 관절값과 Depth를 함께 기록합니다. 서로 다른 위치를 6개 이상 기록하세요.</div><button onclick="recordIkPoint()">현재 3D 좌표 + 관절값 기록</button><button onclick="clearIk()">IK 기록 삭제</button><div class="line" id="ikPoints">IK 보정점 0개</div></fieldset>
<fieldset><legend>동작 설정</legend><label>구간 이동시간(초)<input id="seconds" type="number" min="0.5" max="10" step="0.5" value="2.5"></label><label>그리퍼 열림 각도<input id="open" type="number" min="0" max="180" value="90"></label><label>그리퍼 닫힘 각도<input id="closed" type="number" min="0" max="180" value="30"></label><button class="primary" onclick="run()">픽 → 드롭 실행</button><button class="danger" onclick="post('/api/stop',{})">정지</button></fieldset>
<div class="status" id="status">연결 중…</div>
</aside></div><script>
const cam=document.getElementById('cam'),cam2=document.getElementById('cam2'),selected=document.getElementById('selected'),selected2=document.getElementById('selected2'),pick=document.getElementById('pick'),drop=document.getElementById('drop'),ikPoints=document.getElementById('ikPoints'),captures=document.getElementById('captures'),status=document.getElementById('status'),cameraState=document.getElementById('cameraState'),camera2State=document.getElementById('camera2State'),joints=document.getElementById('joints'),seconds=document.getElementById('seconds'),openAngle=document.getElementById('open'),closedAngle=document.getElementById('closed');
async function post(url,data){try{let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});let j=await r.json();if(!r.ok)throw Error(j.error||'요청 실패');status.textContent=j.status||'완료';return j}catch(e){status.textContent='오류: '+e.message}}
cam.onclick=async e=>{let r=cam.getBoundingClientRect();await post('/api/select',{u:(e.clientX-r.left)*cam.naturalWidth/r.width,v:(e.clientY-r.top)*cam.naturalHeight/r.height})};
cam2.onclick=async e=>{let r=cam2.getBoundingClientRect();await post('/api/select/usb',{u:(e.clientX-r.left)*cam2.naturalWidth/r.width,v:(e.clientY-r.top)*cam2.naturalHeight/r.height})};
function torque(v){post('/api/torque',{enabled:v})}
function target(v){post('/api/target',{target:v})}
function recordIkPoint(){post('/api/ik/record',{})} function clearIk(){if(confirm('IK 보정 기록을 삭제할까요?'))post('/api/ik/clear',{})}
async function capturePair(){try{status.textContent='두 카메라를 촬영하는 중…';let r=await fetch('/api/capture/download',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});if(!r.ok){let j=await r.json();throw Error(j.error||'촬영 실패')}let blob=await r.blob(),name=(r.headers.get('Content-Disposition')||'').match(/filename="?([^";]+)"?/),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name?name[1]:'dofbot_capture.zip';document.body.appendChild(a);a.click();a.remove();URL.revokeObjectURL(a.href);status.textContent='PC 다운로드를 시작했습니다.'}catch(e){status.textContent='오류: '+e.message}}
function detectHandle(){post('/api/detect',{kind:'handle'})}
function detectDrop(){post('/api/detect',{kind:'drop'})}
async function reconnectCamera(){cam.src='';await post('/api/camera/reconnect',{});setTimeout(()=>{cam.src='/stream?t='+Date.now()},3000)}
async function reconnectUsbCamera(){cam2.src='';await post('/api/camera/usb/reconnect',{});setTimeout(()=>{cam2.src='/stream/usb?t='+Date.now()},1500)}
function clearCal(){if(confirm('보정점을 모두 삭제할까요?'))post('/api/clear',{})}
function run(){post('/api/run',{seconds:+seconds.value,open:+openAngle.value,closed:+closedAngle.value})}
function fmt(p){return p?`u=${p[0].toFixed(0)}, v=${p[1].toFixed(0)}, depth=${p[2].toFixed(0)} mm`:'—'}
async function poll(){try{let s=await(await fetch('/api/status')).json();selected.textContent='RGB-D 선택: '+fmt(s.selected);selected2.textContent='C270 선택: '+(s.usb_selected?`u=${s.usb_selected[0].toFixed(0)}, v=${s.usb_selected[1].toFixed(0)}`:'—');pick.textContent='픽 위치: '+fmt(s.pick);drop.textContent='드롭 위치: '+fmt(s.drop);ikPoints.textContent=`IK 보정점 ${s.ik_points}/6${s.ik_points>=6?' (기록 완료)':''}`;captures.textContent=`PC 촬영 다운로드 ${s.download_count}세트`;joints.textContent=s.angles.map((v,i)=>`J${i+1} ${v===null?'—':v.toFixed(1)+'°'}`).join(' · ');status.textContent=s.status;cameraState.textContent=s.camera?`RGB-D: ${s.width}×${s.height} 수신 중 (${s.frame_age.toFixed(1)}초 전)`:'RGB-D: 영상 없음';camera2State.textContent=s.usb_camera?`C270: ${s.usb_width}×${s.usb_height} 수신 중 (${s.usb_frame_age.toFixed(1)}초 전)`:`C270: 영상 없음 (${s.usb_error||'/dev/video0 확인'})`;}catch(e){status.textContent='서버 연결 끊김'}setTimeout(poll,700)}poll();
</script></body></html>'''


XYZ_PAGE = r'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DOFBOT 평면 XYZ 보정</title><style>
:root{font-family:-apple-system,BlinkMacSystemFont,"Noto Sans KR",sans-serif;color:#172033;background:#eef2f7}*{box-sizing:border-box}body{margin:0;padding:18px}.wrap{max-width:1180px;margin:auto;display:grid;grid-template-columns:minmax(480px,2fr) minmax(340px,1fr);gap:16px}.card{background:#fff;border-radius:14px;box-shadow:0 3px 14px #17203318;padding:14px}h1{font-size:22px;margin:0 0 6px}.muted{color:#657086;font-size:13px}.camera{position:relative;background:#222;min-height:360px;border-radius:9px;overflow:hidden}.camera img{display:block;width:100%;height:auto;cursor:crosshair}.line{padding:7px 3px}.value{font-family:ui-monospace,SFMono-Regular,monospace;background:#f1f5f9;border-radius:7px;padding:8px;margin:5px 0}fieldset{border:1px solid #dbe2ec;border-radius:10px;margin:12px 0;padding:10px}legend{font-weight:700}button{border:0;border-radius:8px;padding:9px 11px;margin:3px;background:#e8edf5;font-weight:650;cursor:pointer}button.primary{background:#2563eb;color:#fff}button.danger{background:#dc2626;color:#fff}input{width:100px;padding:7px;border:1px solid #ccd5e2;border-radius:6px}.status{background:#f1f5f9;border-radius:8px;padding:10px}.good{color:#087f5b}.bad{color:#b42318}a{color:#2563eb}@media(max-width:850px){.wrap{grid-template-columns:1fr}.camera{min-height:240px}}
</style></head><body><div class="wrap"><section class="card"><h1>평면 XY + 고정 Z 보정</h1><div class="muted">영상의 실제 기준점을 클릭한 뒤 그 위치에 그리퍼 끝을 맞추고 보정점을 기록하세요.</div><div class="camera"><img id="cam" src="/stream" alt="RGB-D"></div><div class="value" id="pixel">픽셀: —</div><div class="value" id="cameraXYZ">카메라 XYZ: —</div><button class="primary" onclick="setTarget()">현재 클릭을 시험 목표로 설정</button><div class="value" id="targetState">고정된 시험 목표: —</div><div class="value" id="robotXYZ">예상 로봇 XYZ: —</div><div class="line"><a href="/">← 기존 카메라/로봇 GUI</a></div></section><aside class="card">
<fieldset><legend>현재 로봇 끝점</legend><div class="value" id="currentRobot">XYZ: —</div><div class="muted">현재 관절값에서 FK로 계산한 그리퍼 좌표입니다.</div></fieldset>
<fieldset><legend>로봇 토크</legend><button class="primary" onclick="torque(true)">Torque ON</button><button class="danger" onclick="torque(false)">Torque OFF</button><div class="muted">OFF를 누르면 팔이 즉시 처질 수 있습니다. 반드시 팔을 손으로 받친 상태에서 사용하세요.</div></fieldset>
<fieldset><legend>XY 평면 보정</legend><button class="primary" onclick="recordPoint()">현재 클릭 + 로봇 XYZ 기록</button><button onclick="clearPoints()">보정점 삭제</button><div class="line" id="pointCount">보정점 0/4</div><div class="line" id="fitError">보정 오차: —</div><div class="muted">작업 영역의 모서리와 중앙을 포함해 6~10점을 고르게 기록하세요.</div></fieldset>
<fieldset><legend>2개 층 고정 픽 높이</legend><label>1층 로봇 Z (cm) <input id="fixedZ1" type="number" step="0.1"></label><button onclick="saveZ(1)">1층 Z 저장</button><button onclick="useCurrentZ(1)">현재 Z를 1층으로</button><label>2층 로봇 Z (cm) <input id="fixedZ2" type="number" step="0.1"></label><button onclick="saveZ(2)">2층 Z 저장</button><button onclick="useCurrentZ(2)">현재 Z를 2층으로</button><div class="line"><button id="layer1" onclick="selectLayer(1)">1층 선택</button><button id="layer2" onclick="selectLayer(2)">2층 선택</button></div><div class="muted">상자 높이에 맞는 층을 선택하면 예상 로봇 XYZ에 해당 Z가 적용됩니다.</div></fieldset>
<fieldset><legend>단계별 저속 시험</legend><label>접근/상승 높이 (cm) <input id="approach" type="number" min="2" max="12" step="0.5" value="5"></label><label>이동시간 (초) <input id="moveSeconds" type="number" min="1" max="10" step="0.5" value="3"></label><label>그리퍼 열림 J6 (°) <input id="openAngle" type="number" min="0" max="180" value="90"></label><label>그리퍼 닫힘 J6 (°) <input id="closeAngle" type="number" min="0" max="180" value="30"></label><div class="line"><button class="primary" onclick="moveStage('above')">① 목표 위로 이동</button><button onclick="gripper('open')">② 그리퍼 열기</button><button onclick="moveStage('down')">③ 저속 하강</button><button onclick="gripper('close')">④ 그리퍼 닫기</button><button onclick="moveStage('lift')">⑤ 수직 상승</button><button onclick="gripper('open')">⑥ 그리퍼 열기</button><button class="danger" onclick="stopRobot()">■ 정지</button></div><div class="muted">각 단계 후 자세를 확인하고 다음 버튼을 누르세요. 보정 전이거나 범위 밖 목표면 이동이 차단됩니다.</div></fieldset>
<fieldset><legend>안전 확인</legend><div class="line" id="inside">보정 영역: —</div><div class="line" id="depthCheck">Depth 높이: —</div><div class="muted">이 화면은 좌표 보정·미리보기용이며 로봇을 자동으로 움지이지 않습니다.</div></fieldset><div class="status" id="status">연결 중…</div></aside></div><script>
const cam=document.getElementById('cam'),pixel=document.getElementById('pixel'),cameraXYZ=document.getElementById('cameraXYZ'),targetState=document.getElementById('targetState'),robotXYZ=document.getElementById('robotXYZ'),currentRobot=document.getElementById('currentRobot'),pointCount=document.getElementById('pointCount'),fitError=document.getElementById('fitError'),fixedZ1=document.getElementById('fixedZ1'),fixedZ2=document.getElementById('fixedZ2'),layer1=document.getElementById('layer1'),layer2=document.getElementById('layer2'),approach=document.getElementById('approach'),moveSeconds=document.getElementById('moveSeconds'),openAngle=document.getElementById('openAngle'),closeAngle=document.getElementById('closeAngle'),inside=document.getElementById('inside'),depthCheck=document.getElementById('depthCheck'),status=document.getElementById('status');
let localMessageUntil=0;
async function post(url,data={}){let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}),j=await r.json();if(!r.ok){status.textContent='오류: '+(j.error||'요청 실패');localMessageUntil=Date.now()+8000;throw Error(j.error||'요청 실패')}status.textContent=j.status||'완료';localMessageUntil=Date.now()+3000;return j}
cam.onclick=async e=>{let r=cam.getBoundingClientRect();try{await post('/api/select',{u:(e.clientX-r.left)*cam.naturalWidth/r.width,v:(e.clientY-r.top)*cam.naturalHeight/r.height})}catch(x){status.textContent='오류: '+x.message}};
async function recordPoint(){try{await post('/api/xyz/record')}catch(e){status.textContent='오류: '+e.message}}
async function clearPoints(){if(confirm('XY 보정점을 모두 삭제할까요?'))try{await post('/api/xyz/clear')}catch(e){status.textContent='오류: '+e.message}}
async function saveZ(layer){try{await post('/api/xyz/fixed-z',{layer,z:+(layer===1?fixedZ1.value:fixedZ2.value)})}catch(e){status.textContent='오류: '+e.message}}
async function useCurrentZ(layer){try{let r=await post('/api/xyz/use-current-z',{layer});(layer===1?fixedZ1:fixedZ2).value=r.fixed_z.toFixed(2)}catch(e){status.textContent='오류: '+e.message}}
async function selectLayer(layer){try{await post('/api/xyz/layer',{layer})}catch(e){status.textContent='오류: '+e.message}}
async function setTarget(){try{await post('/api/xyz/target')}catch(e){status.textContent='오류: '+e.message}}
async function torque(enabled){if(!enabled&&!confirm('팔을 손으로 받치고 있습니까? Torque OFF를 실행할까요?'))return;try{await post('/api/torque',{enabled})}catch(e){status.textContent='오류: '+e.message}}
async function moveStage(stage){if(!confirm(stage==='down'?'선택한 손잡이 Z까지 저속 하강할까요?':'로봇을 선택 목표로 이동할까요?'))return;try{await post('/api/xyz/move',{stage,approach:+approach.value,seconds:+moveSeconds.value})}catch(e){status.textContent='오류: '+e.message}}
async function gripper(action){try{await post('/api/xyz/gripper',{action,open:+openAngle.value,closed:+closeAngle.value,seconds:1})}catch(e){status.textContent='오류: '+e.message}}
async function stopRobot(){try{await post('/api/stop')}catch(e){status.textContent='오류: '+e.message}}
function xyz(v,unit){return v?`X=${v[0].toFixed(2)}, Y=${v[1].toFixed(2)}, Z=${v[2].toFixed(2)} ${unit}`:'—'}
async function poll(){try{let s=await(await fetch('/api/xyz/status')).json();pixel.textContent=s.selected?`RGB-D 픽셀: u=${s.selected[0].toFixed(1)}, v=${s.selected[1].toFixed(1)}, depth=${s.selected[2].toFixed(0)} mm`:'RGB-D 픽셀: —';cameraXYZ.textContent='카메라 XYZ: '+xyz(s.camera_xyz,'mm');targetState.textContent=s.target_pixel?`고정된 시험 목표: u=${s.target_pixel[0].toFixed(1)}, v=${s.target_pixel[1].toFixed(1)}`:'고정된 시험 목표: —';robotXYZ.textContent=`예상 로봇 XYZ (${s.active_layer}층): `+xyz(s.robot_xyz,'cm');currentRobot.textContent='현재 로봇 XYZ: '+xyz(s.current_robot_xyz,'cm');pointCount.textContent=`보정점 ${s.point_count}/4${s.calibrated?' (사용 가능)':''}`;fitError.textContent=s.calibration_error?`보정 오류: ${s.calibration_error}`:s.fit_error_cm===null?'보정 오차: —':`보정 평균오차: ${s.fit_error_cm.toFixed(2)} cm`;fitError.className='line '+(s.calibration_error?'bad':'');if(document.activeElement!==fixedZ1)fixedZ1.value=s.fixed_z_1.toFixed(2);if(document.activeElement!==fixedZ2)fixedZ2.value=s.fixed_z_2.toFixed(2);layer1.className=s.active_layer===1?'primary':'';layer2.className=s.active_layer===2?'primary':'';inside.textContent='현재 클릭: '+(s.selected_inside===null?'—':s.selected_inside?'보정 영역 내부':'보정 영역 밖 ⚠');inside.className='line '+(s.selected_inside===false?'bad':s.selected_inside?'good':'');depthCheck.textContent=s.depth_delta_mm===null?'Depth 높이: —':`Depth 기준 차이: ${s.depth_delta_mm.toFixed(0)} mm`;if(Date.now()>localMessageUntil)status.textContent=s.status}catch(e){if(Date.now()>localMessageUntil)status.textContent='서버 연결 끊김'}setTimeout(poll,600)}poll();
</script></body></html>'''

HANDLE_PAGE = r'''<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>RGB-D 손잡이 필터</title><style>
:root{font-family:-apple-system,BlinkMacSystemFont,"Noto Sans KR",sans-serif;color:#172033;background:#eef2f7}*{box-sizing:border-box}body{margin:0;padding:16px}.wrap{max-width:1300px;margin:auto}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.card{background:#fff;border-radius:14px;padding:14px;box-shadow:0 3px 14px #17203318;margin-bottom:12px}.view{background:#222;border-radius:9px;overflow:hidden;min-height:260px}.view img{display:block;width:100%;height:auto}h1{margin:0 0 5px;font-size:22px}h2{font-size:16px;margin:0 0 7px}.muted{font-size:13px;color:#657086}.controls{display:grid;grid-template-columns:repeat(4,minmax(150px,1fr));gap:9px}.controls label{display:flex;flex-direction:column;gap:4px;font-size:13px}.controls input{padding:7px;border:1px solid #ccd5e2;border-radius:6px}button{border:0;border-radius:8px;padding:9px 12px;background:#e8edf5;font-weight:650;cursor:pointer}.primary{background:#2563eb;color:#fff}.value{font-family:ui-monospace,SFMono-Regular,monospace;background:#f1f5f9;padding:8px;border-radius:7px;margin:5px 0}.good{color:#087f5b}.bad{color:#b42318}.status{padding:10px;border-radius:8px;background:#f1f5f9}a{color:#2563eb}@media(max-width:850px){.grid,.controls{grid-template-columns:1fr}.view{min-height:220px}}</style></head><body><div class="wrap"><div class="card"><h1>RGB-D 검은 손잡이 검출</h1><div class="muted">색상 + 형태 + Depth 조건을 모두 통과한 후보만 표시합니다. <a href="/xyz">XYZ 보정 GUI</a> · <a href="/">메인 GUI</a></div></div><div class="grid"><div class="card"><h2>검출 오버레이</h2><div class="view"><img src="/stream/handle" alt="handle detection"></div></div><div class="card"><h2>검은색 마스크</h2><div class="view"><img src="/stream/handle-mask" alt="handle mask"></div></div></div><div class="card"><div class="controls"><label>최대 밝기 V (0~255)<input id="vmax" type="number" min="20" max="180"></label><label>최소 면적 px<input id="amin" type="number" min="10" max="5000"></label><label>최대 면적 px<input id="amax" type="number" min="100" max="30000"></label><label>최소 길쭉함<input id="aspect" type="number" min="1" max="10" step="0.1"></label><label>최소 Depth mm<input id="dmin" type="number" min="100" max="3000"></label><label>최대 Depth mm<input id="dmax" type="number" min="100" max="4000"></label><label>안정 프레임 수<input id="stable" type="number" min="1" max="20"></label><label>중심 안정 오차 px<input id="jitter" type="number" min="1" max="80"></label></div><div style="margin-top:10px"><button class="primary" onclick="applyFilter()">필터 적용</button> <button onclick="setTarget()">최적 손잡이를 시험 목표로</button></div></div><div class="card"><div class="value" id="det">검출: —</div><div class="value" id="camxyz">카메라 XYZ: —</div><div class="value" id="robotxyz">로봇 XYZ: —</div><div class="value" id="stableState">안정성: —</div><div class="status" id="status">연결 중…</div></div></div><script>
const ids=['vmax','amin','amax','aspect','dmin','dmax','stable','jitter'],els=Object.fromEntries(ids.map(x=>[x,document.getElementById(x)])),det=document.getElementById('det'),camxyz=document.getElementById('camxyz'),robotxyz=document.getElementById('robotxyz'),stableState=document.getElementById('stableState'),status=document.getElementById('status');let editing=false;ids.forEach(x=>els[x].onfocus=()=>editing=true);ids.forEach(x=>els[x].onblur=()=>editing=false);
async function post(url,data){let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}),j=await r.json();if(!r.ok)throw Error(j.error||'요청 실패');status.textContent=j.status||'완료';return j}async function applyFilter(){try{await post('/api/handle/config',{vmax:+els.vmax.value,area_min:+els.amin.value,area_max:+els.amax.value,aspect_min:+els.aspect.value,depth_min:+els.dmin.value,depth_max:+els.dmax.value,stable_frames:+els.stable.value,jitter_px:+els.jitter.value})}catch(e){status.textContent='오류: '+e.message}}async function setTarget(){try{await post('/api/handle/target',{})}catch(e){status.textContent='오류: '+e.message}}function xyz(v,u){return v?`X=${v[0].toFixed(1)}, Y=${v[1].toFixed(1)}, Z=${v[2].toFixed(1)} ${u}`:'—'}async function poll(){try{let s=await(await fetch('/api/handle/status')).json(),b=s.best;if(!editing){let c=s.config;els.vmax.value=c.vmax;els.amin.value=c.area_min;els.amax.value=c.area_max;els.aspect.value=c.aspect_min;els.dmin.value=c.depth_min;els.dmax.value=c.depth_max;els.stable.value=c.stable_frames;els.jitter.value=c.jitter_px}det.textContent=b?`검출: u=${b.u.toFixed(1)}, v=${b.v.toFixed(1)}, angle=${b.angle.toFixed(1)}°, area=${b.area.toFixed(0)}, depth=${b.depth.toFixed(0)}mm`:'검출: 조건을 통과한 손잡이 없음';camxyz.textContent='카메라 XYZ: '+xyz(s.camera_xyz,'mm');robotxyz.textContent='예상 로봇 XYZ: '+xyz(s.robot_xyz,'cm');stableState.textContent=`안정성: ${s.stable_count}/${s.config.stable_frames} ${s.stable?'✓ 목표 사용 가능':'대기'}`;stableState.className='value '+(s.stable?'good':'');status.textContent=s.status}catch(e){status.textContent='서버 연결 끊김'}setTimeout(poll,500)}poll();
</script></body></html>'''


# Extend the handle detector page with guarded, step-by-step pick controls.
HANDLE_PAGE = HANDLE_PAGE.replace('RGB-D 검은 손잡이 검출', 'RGB-D 파란 손잡이 검출')
HANDLE_PAGE = HANDLE_PAGE.replace('검은색 마스크', '파란색 마스크')
HANDLE_PAGE = HANDLE_PAGE.replace('색상 + 형태 + Depth 조건', 'Blur 5×5 + 파란색 + 형태 + Depth 조건')
HANDLE_PAGE = HANDLE_PAGE.replace('최대 밝기 V (0~255)', '최소 채도 S (0~255)')
HANDLE_PAGE = HANDLE_PAGE.replace(
    '<button class="primary" onclick="applyFilter()">필터 적용</button>',
    '<button class="primary" onclick="applyFilter()">필터 적용 ON</button> <button onclick="disableFilter()">필터 해제 OFF</button> <span id="filterState">필터 상태 확인 중…</span>')
for _filter_id in ('vmax', 'amin', 'amax', 'aspect', 'dmin', 'dmax', 'stable', 'jitter'):
    HANDLE_PAGE = HANDLE_PAGE.replace(
        f'<input id="{_filter_id}" type="number"',
        f'<input id="{_filter_id}" type="range" oninput="document.getElementById(\'{_filter_id}Value\').textContent=this.value"')
    HANDLE_PAGE = HANDLE_PAGE.replace(
        f'<input id="{_filter_id}" type="range"',
        f'<span id="{_filter_id}Value">—</span><input id="{_filter_id}" type="range"')
HANDLE_PAGE = HANDLE_PAGE.replace(
    '<div class="card"><div class="value" id="det">',
    '''<div class="card"><h2>손잡이 단계별 집기</h2>
    <div class="controls"><label>접근 높이 cm<input id="hApproach" type="number" value="3" min="1" max="10" step="0.5"></label><label>이동 시간 초<input id="hSeconds" type="number" value="3" min="1" max="10" step="0.5"></label><label>그리퍼 열림 각도<input id="hOpen" type="number" value="90" min="0" max="180"></label><label>그리퍼 닫힘 각도<input id="hClosed" type="number" value="30" min="0" max="180"></label></div>
    <div style="margin-top:10px"><button onclick="chooseLayer(1)">1층</button> <button onclick="chooseLayer(2)">2층</button> <button onclick="torque(true)">토크 ON</button> <button onclick="torque(false)">토크 OFF</button></div>
    <div style="margin-top:10px"><button class="primary" onclick="moveHandle('above')">① 목표 위로</button> <button onclick="gripper('open')">② 그리퍼 열기</button> <button onclick="moveHandle('down')">③ 하강</button> <button onclick="gripper('close')">④ 그리퍼 닫기</button> <button onclick="moveHandle('lift')">⑤ 상승</button> <button class="bad" onclick="stopRobot()">정지</button></div>
    <div class="muted" style="margin-top:8px">먼저 ‘최적 손잡이를 시험 목표로’를 눌러 목표를 고정한 뒤 단계별로 시험하세요.</div></div>
    <div class="card"><div class="value" id="det">''')
HANDLE_PAGE = HANDLE_PAGE.replace(
    '<img src="/stream/handle" alt="handle detection">',
    '<img id="handleCam" src="/stream/handle" alt="handle detection" onclick="selectHandle(event)" style="cursor:crosshair">')
HANDLE_PAGE = HANDLE_PAGE.replace(
    '<div class="card"><h2>손잡이 단계별 집기</h2>',
    '''<div class="card"><h2>화면+Depth→관절 3D 직접 보정</h2><div class="muted">보정 모드에서는 필터와 관계없이 영상 아무 곳이나 클릭할 수 있습니다. Depth가 0이 아닌 점만 기록하며, 거리별 보정점을 사용합니다.</div><div style="margin-top:8px"><label style="display:inline-flex;gap:6px;align-items:center"><input id="calMode" type="checkbox" style="width:auto"> 보정용 자유 클릭 모드</label> <label>보정점 이름 <input id="jointLabel" value="P1" style="width:90px;padding:7px"></label> <button onclick="recordJoint('above')">위 자세 기록</button> <button onclick="recordJoint('down')">집기 자세 기록</button> <button onclick="deleteJoint()">현재 이름 삭제</button> <button onclick="clearJoint()">현재 층 전체 삭제</button></div><div class="value" id="calPoint">CAL POINT: —</div><div class="value" id="jointState">3D 관절 보정: 0쌍</div></div><div class="card"><h2>손잡이 단계별 집기 · 3D 관절 직접 보간</h2>''')
HANDLE_PAGE = HANDLE_PAGE.replace(
    '<div class="card"><div class="value" id="det">',
    '''<div class="card"><h2>고정 드롭 위치 · 연속 동작</h2><div class="muted">컨베이어의 고정 드롭 위치에서 팔을 직접 맞춰 두 자세를 저장합니다.</div><label>드롭 장거리 이동 시간 <span id="dropSecondsValue">6</span>초<input id="dropSeconds" type="range" min="3" max="15" step="0.5" value="6" oninput="dropSecondsValue.textContent=this.value"></label><div style="margin-top:8px"><button onclick="recordDrop('above')">드롭 위 자세 기록</button> <button onclick="recordDrop('down')">드롭 놓기 자세 기록</button> <button class="primary" onclick="runPickDrop()">▶ 집기→드롭 연속 실행</button></div><div class="value" id="dropState">드롭 보정: —</div></div><div class="card"><div class="value" id="det">''')
HANDLE_PAGE = HANDLE_PAGE.replace(
    '<div class="value" id="stableState">',
    '<div class="value" id="targetInfo">고정 목표: —</div><div class="value" id="calibrationState">XY 보정: 확인 중…</div><div class="value" id="stableState">')
HANDLE_PAGE = HANDLE_PAGE.replace(
    "async function setTarget(){try{await post('/api/handle/target',{})}catch(e){status.textContent='오류: '+e.message}}",
    """async function disableFilter(){try{await post('/api/handle/filter-enabled',{enabled:false})}catch(e){status.textContent='오류: '+e.message}}
async function setTarget(){try{await post('/api/handle/target',{})}catch(e){status.textContent='오류: '+e.message}}
async function recordJoint(kind){try{await post('/api/joint/record',{kind:kind,label:document.getElementById('jointLabel').value.trim()||'P1'})}catch(e){status.textContent='오류: '+e.message}}
async function clearJoint(){if(confirm('화면→관절 보정 기록을 모두 삭제할까요?'))try{await post('/api/joint/clear',{})}catch(e){status.textContent='오류: '+e.message}}
async function deleteJoint(){let label=document.getElementById('jointLabel').value.trim();if(!label||!confirm(`${label} 보정점을 삭제할까요?`))return;try{await post('/api/joint/delete',{labels:[label]})}catch(e){status.textContent='오류: '+e.message}}
async function recordDrop(kind){try{await post('/api/joint/drop/record',{kind:kind})}catch(e){status.textContent='오류: '+e.message}}
async function runPickDrop(){if(!confirm('현재 고정 목표를 집어 고정 드롭 위치까지 연속 실행할까요?'))return;try{await post('/api/joint/run',{seconds:+document.getElementById('hSeconds').value,drop_seconds:+document.getElementById('dropSeconds').value,open:+document.getElementById('hOpen').value,closed:+document.getElementById('hClosed').value})}catch(e){status.textContent='오류: '+e.message}}
async function chooseLayer(layer){try{await post('/api/xyz/layer',{layer:layer})}catch(e){status.textContent='오류: '+e.message}}
async function torque(on){try{await post('/api/torque',{on:on})}catch(e){status.textContent='오류: '+e.message}}
async function moveHandle(stage){try{let grip=stage==='lift'?+document.getElementById('hClosed').value:+document.getElementById('hOpen').value;await post('/api/joint/move',{stage:stage,seconds:+document.getElementById('hSeconds').value,gripper:grip})}catch(e){status.textContent='오류: '+e.message}}
async function gripper(action){try{await post('/api/xyz/gripper',{action:action,open:+document.getElementById('hOpen').value,closed:+document.getElementById('hClosed').value,seconds:1})}catch(e){status.textContent='오류: '+e.message}}
async function stopRobot(){try{await post('/api/stop',{})}catch(e){status.textContent='오류: '+e.message}}""")
HANDLE_PAGE = HANDLE_PAGE.replace(
    'async function chooseLayer(layer)',
    """async function selectHandle(e){let im=e.currentTarget,r=im.getBoundingClientRect(),u=(e.clientX-r.left)*im.naturalWidth/r.width,v=(e.clientY-r.top)*im.naturalHeight/r.height,url=document.getElementById('calMode').checked?'/api/joint/select':'/api/handle/select';try{await post(url,{u:u,v:v})}catch(x){status.textContent='오류: '+x.message}}
async function chooseLayer(layer)""")
HANDLE_PAGE = HANDLE_PAGE.replace(
    "stableState.className='value '+(s.stable?'good':'');status.textContent=s.status",
    "stableState.className='value '+(s.stable?'good':'');document.getElementById('targetInfo').textContent=s.target_pixel?`고정 목표: u=${s.target_pixel[0].toFixed(1)}, v=${s.target_pixel[1].toFixed(1)} / ${s.active_layer}층 Z=${s.fixed_z.toFixed(1)}cm`:'고정 목표: —';let cs=document.getElementById('calibrationState');cs.textContent=s.calibrated?'XY 보정: 사용 가능':`XY 보정 필요: ${s.calibration_error||'보정점 부족'}`;cs.className='value '+(s.calibrated?'good':'bad');status.textContent=s.status")
HANDLE_PAGE = HANDLE_PAGE.replace(
    ";status.textContent=s.status}catch(e){status.textContent='서버 연결 끊김'}",
    ";let cp=document.getElementById('calPoint');cp.textContent=s.selected?`CAL POINT: u=${s.selected[0].toFixed(1)}, v=${s.selected[1].toFixed(1)}, depth=${s.selected[2].toFixed(0)}mm`:'CAL POINT: —';let js=document.getElementById('jointState');js.textContent=`${s.active_layer}층 3D 관절 보정: 완료 ${s.joint_complete}쌍 / 위 자세만 ${s.joint_partial}개 / Depth ${s.joint_depth_min===null?'—':s.joint_depth_min.toFixed(0)+'~'+s.joint_depth_max.toFixed(0)+'mm'}`;js.className='value '+(s.joint_ready?'good':'bad');let ds=document.getElementById('dropState');ds.textContent=`드롭 보정: 위 ${s.drop_above?'✓':'—'} / 놓기 ${s.drop_down?'✓':'—'}`;ds.className='value '+(s.drop_ready?'good':'bad');let cs2=document.getElementById('calibrationState');cs2.textContent=s.joint_ready?'이동 방식: (u,v,Depth) 3D 관절 보간':'이동 차단: 현재 층 유효 Depth 보정 3쌍 이상 필요';cs2.className='value '+(s.joint_ready?'good':'bad');status.textContent=s.status}catch(e){status.textContent='서버 연결 끊김'}")
HANDLE_PAGE = HANDLE_PAGE.replace(
    'els.jitter.value=c.jitter_px}',
    "els.jitter.value=c.jitter_px;ids.forEach(x=>document.getElementById(x+'Value').textContent=els[x].value)}")
HANDLE_PAGE = HANDLE_PAGE.replace(
    'det.textContent=b?',
    "document.getElementById('filterState').textContent=s.filter_enabled?'필터 ON':'필터 OFF';det.textContent=b?")


class UsbCamera:
    """Continuously capture the auxiliary UVC camera without blocking ROS."""
    def __init__(self, device='/dev/video0', width=640, height=480, fps=10):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.lock = threading.RLock()
        self.jpeg = None
        self.clean_jpeg = None
        self.frame_size = (0, 0)
        self.selected = None
        self.last_frame_received = 0.0
        self.error = '시작 중'
        self.reconnect_event = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            if not cap.isOpened():
                with self.lock:
                    self.error = f'{self.device} 열기 실패'
                cap.release()
                time.sleep(2.0)
                continue
            self.reconnect_event.clear()
            with self.lock:
                self.error = ''
            while not self.reconnect_event.is_set():
                ok, frame = cap.read()
                if not ok:
                    with self.lock:
                        self.error = '프레임 수신 실패'
                    break
                clean_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])[1]
                with self.lock:
                    selected = self.selected
                if selected is not None:
                    x, y = round(selected[0]), round(selected[1])
                    cv2.drawMarker(frame, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 26, 2)
                    cv2.putText(frame, 'SELECTED', (x + 10, y + 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with self.lock:
                        self.jpeg = bytes(encoded)
                        self.clean_jpeg = bytes(clean_encoded)
                        self.frame_size = (frame.shape[1], frame.shape[0])
                        self.last_frame_received = time.monotonic()
                        self.error = ''
            cap.release()
            with self.lock:
                self.jpeg = None
            time.sleep(0.5)

    def reconnect(self):
        with self.lock:
            self.jpeg = None
            self.last_frame_received = 0.0
            self.error = '다시 연결하는 중'
        self.reconnect_event.set()

    def select(self, u, v):
        with self.lock:
            width, height = self.frame_size
            if width <= 0 or height <= 0:
                raise ValueError('C270 카메라 영상이 아직 없습니다.')
            self.selected = (max(0.0, min(width - 1, float(u))),
                             max(0.0, min(height - 1, float(v))))
            return self.selected

    def state(self):
        with self.lock:
            age = time.monotonic() - self.last_frame_received if self.last_frame_received else 0.0
            width, height = self.frame_size
            return self.jpeg is not None and age < 3.0, age, width, height, self.error, self.selected


class Controller(Node):
    def __init__(self) -> None:
        super().__init__('dofbot_camera_coordinate_web')
        self.lock = threading.RLock()
        self.rgb = None
        self.depth = None
        self.jpeg = None
        self.angles = [None] * 6
        self.selected = None
        self.pick = None
        self.drop = None
        self.ik_points = []
        self.samples = []
        self.status = '카메라와 팔 드라이버를 기다리는 중입니다.'
        self.playing = False
        self.calibration_mode = 'pick'
        self.stop_event = threading.Event()
        self.last_rgb_time = 0.0
        self.last_depth_time = 0.0
        self.last_frame_received = 0.0
        self.usb_camera = UsbCamera(os.environ.get('DOFBOT_USB_CAMERA', '/dev/video0'))
        self.capture_count = len(list((DATASET_DIR / 'meta').glob('capture_*.json')))
        self.download_count = 0
        self.camera_intrinsics = None
        self.handle_filter = {'vmax': 90, 'area_min': 40, 'area_max': 3500,
                              'aspect_min': 1.25, 'depth_min': 250, 'depth_max': 1600,
                              'stable_frames': 5, 'jitter_px': 18}
        self.handle_filter_enabled = True
        self.handle_detections = []
        self.handle_best = None
        self.handle_stable_count = 0
        self.handle_previous_center = None
        self.handle_jpeg = None
        self.handle_mask_jpeg = None
        self.planar_points = []
        self.fixed_pick_zs = {1: 5.0, 2: 5.0}
        self.active_layer = 1
        self.test_target_pixel = None
        self.load_planar_calibration()
        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/arm/joint_angles', self.on_angles, 10)
        self.move_pub = self.create_publisher(Float32MultiArray, '/arm/move_all', 10)
        self.torque_pub = self.create_publisher(Bool, '/arm/torque_cmd', 10)
        self.load()
        self.load_ik_points()

    def on_camera_info(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0 and msg.k[4] > 0:
            with self.lock:
                self.camera_intrinsics = (float(msg.k[0]), float(msg.k[4]),
                                          float(msg.k[2]), float(msg.k[5]))

    def load_planar_calibration(self):
        try:
            data = json.loads(PLANAR_CALIBRATION_FILE.read_text())
            self.planar_points = list(data.get('points', []))
            legacy_z = float(data.get('fixed_pick_z', 5.0))
            saved = data.get('fixed_pick_zs', {})
            self.fixed_pick_zs = {1: float(saved.get('1', legacy_z)),
                                  2: float(saved.get('2', legacy_z))}
            self.active_layer = int(data.get('active_layer', 1))
            if self.active_layer not in (1, 2): self.active_layer = 1
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            self.planar_points = []
            self.fixed_pick_zs = {1: 5.0, 2: 5.0}
            self.active_layer = 1

    def save_planar_calibration(self):
        PLANAR_CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        PLANAR_CALIBRATION_FILE.write_text(json.dumps({
            'fixed_pick_zs': {'1': self.fixed_pick_zs[1], '2': self.fixed_pick_zs[2]},
            'active_layer': self.active_layer,
            'points': self.planar_points,
        }, ensure_ascii=False, indent=2) + '\n')

    def camera_xyz(self, point):
        if point is None or point[2] <= 0 or self.camera_intrinsics is None:
            return None
        fx, fy, cx, cy = self.camera_intrinsics
        u, v, z = map(float, point)
        return [(u - cx) * z / fx, (v - cy) * z / fy, z]

    def current_robot_xyz(self):
        if any(value is None for value in self.angles[:5]):
            return None
        xyz, _ = forward_kinematics_servo(self.angles[:5])
        return [float(value) for value in xyz]

    def planar_model(self):
        if len(self.planar_points) < 4:
            return None, None
        pixels = np.asarray([[p['u'], p['v']] for p in self.planar_points], np.float32)
        robot_xy = np.asarray([[p['robot_x'], p['robot_y']] for p in self.planar_points], np.float32)
        if cv2.contourArea(cv2.convexHull(pixels)) < 400.0:
            return None, None
        if cv2.contourArea(cv2.convexHull(robot_xy)) < 1.0:
            return None, None
        matrix, _ = cv2.findHomography(pixels, robot_xy, method=0)
        if matrix is None:
            return None, None
        predicted = cv2.perspectiveTransform(pixels.reshape(-1, 1, 2), matrix).reshape(-1, 2)
        error = float(np.linalg.norm(predicted - robot_xy, axis=1).mean())
        return matrix, error

    def planar_calibration_error(self):
        if len(self.planar_points) < 4:
            return '보정점이 4개 이상 필요합니다.'
        pixels = np.asarray([[p['u'], p['v']] for p in self.planar_points], np.float32)
        robot_xy = np.asarray([[p['robot_x'], p['robot_y']] for p in self.planar_points], np.float32)
        if cv2.contourArea(cv2.convexHull(pixels)) < 400.0:
            return '카메라 보정점이 한 곳에 몰려 있습니다.'
        if cv2.contourArea(cv2.convexHull(robot_xy)) < 1.0:
            return '로봇 XY 보정점이 일직선입니다. J1을 바꿔 좌우 위치도 기록하세요.'
        matrix, error = self.planar_model()
        if matrix is None:
            return '평면 변환을 계산할 수 없습니다.'
        if error is not None and error > 1.5:
            return f'평균오차 {error:.2f} cm: 1.5 cm 이하로 다시 보정하세요.'
        return None

    def set_planar_target(self):
        with self.lock:
            if self.selected is None or self.selected[2] <= 0:
                raise ValueError('먼저 RGB-D 영상에서 시험 목표를 클릭하세요.')
            error = self.planar_calibration_error()
            if error:
                raise ValueError(error)
            hull = cv2.convexHull(np.asarray([[p['u'], p['v']] for p in self.planar_points], np.float32))
            if cv2.pointPolygonTest(hull, (float(self.selected[0]), float(self.selected[1])), False) < 0:
                raise ValueError('클릭 위치가 XY 보정 영역 밖입니다.')
            self.test_target_pixel = tuple(map(float, self.selected))
            self.status = '현재 클릭을 시험 이동 목표로 고정했습니다.'

    def record_planar_point(self):
        with self.lock:
            if self.selected is None:
                raise ValueError('먼저 RGB-D 영상에서 기준점을 클릭하세요.')
            robot = self.current_robot_xyz()
            if robot is None:
                raise ValueError('현재 로봇 관절값을 읽지 못했습니다.')
            u, v, depth = self.selected
            self.planar_points.append({'u': float(u), 'v': float(v), 'depth': float(depth),
                                       'robot_x': robot[0], 'robot_y': robot[1],
                                       'robot_z': robot[2], 'angles': list(map(float, self.angles))})
            self.save_planar_calibration()
            depth_note = '' if depth > 0 else ' (Depth 없음, XY만 사용)'
            self.status = f'XY 평면 보정점 {len(self.planar_points)}개를 기록했습니다.{depth_note}'

    def planar_state(self):
        with self.lock:
            selected = list(self.selected) if self.selected is not None else None
            robot_current = self.current_robot_xyz()
            camera_xyz = self.camera_xyz(selected)
            matrix, error = self.planar_model()
            calibration_error = self.planar_calibration_error()
            predicted = None
            inside = None
            selected_inside = None
            depth_delta = None
            if selected is not None and len(self.planar_points) >= 3:
                selected_hull = cv2.convexHull(np.asarray(
                    [[p['u'], p['v']] for p in self.planar_points], np.float32))
                selected_inside = cv2.pointPolygonTest(
                    selected_hull, (float(selected[0]), float(selected[1])), False) >= 0
            target = list(self.test_target_pixel) if self.test_target_pixel is not None else None
            if target is not None and matrix is not None:
                xy = cv2.perspectiveTransform(
                    np.asarray([[[target[0], target[1]]]], np.float32), matrix)[0, 0]
                predicted = [float(xy[0]), float(xy[1]), float(self.fixed_pick_zs[self.active_layer])]
                hull = cv2.convexHull(np.asarray([[p['u'], p['v']] for p in self.planar_points], np.float32))
                inside = cv2.pointPolygonTest(hull, (float(target[0]), float(target[1])), False) >= 0
                nearest = min(self.planar_points,
                              key=lambda p: math.hypot(p['u']-target[0], p['v']-target[1]))
                depth_delta = float(target[2] - nearest['depth']) if target[2] > 0 else None
            return {'selected': selected, 'camera_xyz': camera_xyz,
                    'target_pixel': target, 'robot_xyz': predicted, 'current_robot_xyz': robot_current,
                    'point_count': len(self.planar_points), 'calibrated': matrix is not None,
                    'calibration_error': calibration_error,
                    'fit_error_cm': error, 'fixed_z_1': self.fixed_pick_zs[1],
                    'fixed_z_2': self.fixed_pick_zs[2], 'active_layer': self.active_layer,
                    'inside': inside, 'selected_inside': selected_inside,
                    'depth_delta_mm': depth_delta, 'status': self.status}

    def planar_target(self, approach=0.0):
        """Return a validated planar target and refuse unsafe extrapolation."""
        with self.lock:
            if self.test_target_pixel is None:
                raise ValueError("'현재 클릭을 시험 목표로 설정'을 먼저 누르세요.")
            calibration_error = self.planar_calibration_error()
            if calibration_error:
                raise ValueError(calibration_error)
            matrix, error = self.planar_model()
            u, v, _ = self.test_target_pixel
            hull = cv2.convexHull(np.asarray([[p['u'], p['v']] for p in self.planar_points], np.float32))
            if cv2.pointPolygonTest(hull, (float(u), float(v)), False) < 0:
                raise ValueError('목표가 XY 보정 영역 밖입니다.')
            xy = cv2.perspectiveTransform(np.asarray([[[u, v]]], np.float32), matrix)[0, 0]
            z = self.fixed_pick_zs[self.active_layer] + float(approach)
            return [float(xy[0]), float(xy[1]), float(z)]

    def solve_planar_pose(self, target, gripper):
        with self.lock:
            if any(value is None for value in self.angles[:5]):
                raise ValueError('현재 로봇 관절값을 읽지 못했습니다.')
            current = list(self.angles[:5])
        _, rpy = forward_kinematics_servo(current)
        seeds = [current, [90.0, 30.0, 90.0, 0.0, 99.0],
                 [90.0, 0.0, 90.0, 0.0, 99.0]]
        solved = None
        for seed in seeds:
            candidate, ok = inverse_kinematics_servo(
                target, rpy, q_init_servo=seed, rot_weight=0.0,
                tol_rot=10.0, max_iter=1000)
            if ok:
                solved = [float(value) for value in candidate]
                break
        if solved is None:
            raise ValueError(f'IK 계산 실패: 목표 XYZ={np.round(target, 2).tolist()} cm')
        pose = solved + [float(gripper)]
        if any(value < 0 or value > (270 if index == 4 else 180)
               for index, value in enumerate(pose)):
            raise ValueError('계산 자세가 관절 범위를 벗어났습니다.')
        return pose

    def move_planar_stage(self, stage, approach, seconds):
        if stage not in ('above', 'down', 'lift'):
            raise ValueError('알 수 없는 이동 단계입니다.')
        approach = max(2.0, min(12.0, float(approach)))
        seconds = max(1.0, min(10.0, float(seconds)))
        target = self.planar_target(0.0 if stage == 'down' else approach)
        with self.lock:
            gripper = self.angles[5]
            if gripper is None:
                raise ValueError('그리퍼 J6 값을 읽지 못했습니다.')
        pose = self.solve_planar_pose(target, gripper)
        self.torque_pub.publish(Bool(data=True))
        self.move_pub.publish(Float32MultiArray(data=[*pose, float(round(seconds * 1000))]))
        with self.lock:
            label = {'above': '목표 위', 'down': '집기 Z', 'lift': '안전 높이'}[stage]
            self.status = f'{label}로 {seconds:.1f}초 이동 명령을 보냈습니다.'

    def command_gripper(self, action, opened, closed, seconds):
        if action not in ('open', 'close'):
            raise ValueError('알 수 없는 그리퍼 명령입니다.')
        with self.lock:
            if any(value is None for value in self.angles):
                raise ValueError('현재 관절값을 읽지 못했습니다.')
            pose = list(map(float, self.angles))
        pose[5] = max(0.0, min(180.0, float(opened if action == 'open' else closed)))
        seconds = max(0.5, min(3.0, float(seconds)))
        self.torque_pub.publish(Bool(data=True))
        self.move_pub.publish(Float32MultiArray(data=[*pose, float(round(seconds * 1000))]))
        with self.lock:
            self.status = f"그리퍼를 {'열기' if action == 'open' else '닫기'} 명령했습니다."

    def update_handle_detection(self, frame):
        """Detect dark, elongated RGB-D regions and maintain temporal stability."""
        with self.lock:
            config = dict(self.handle_filter)
            filter_enabled = self.handle_filter_enabled
            depth = self.depth.copy() if self.depth is not None else None
            locked_target = self.test_target_pixel
            calibration_selected = self.selected
            direct_points = [(item['u'], item['v']) for item in self.direct_complete()]
        # A light blur suppresses isolated color noise without modifying depth data.
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
        # Blue handle only: OpenCV hue 90..135, with adjustable minimum saturation.
        mask = cv2.inRange(hsv, np.array([90, int(config['vmax']), 40], np.uint8),
                           np.array([135, 255, 255], np.uint8))
        if not filter_enabled:
            mask[:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        height, width = frame.shape[:2]
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not config['area_min'] <= area <= config['area_max']:
                continue
            (cx, cy), (rw, rh), angle = cv2.minAreaRect(contour)
            short, long = sorted((max(rw, 0.1), max(rh, 0.1)))
            aspect = long / short
            if aspect < config['aspect_min']:
                continue
            if rw < rh:
                angle += 90.0
            measured_depth = 0.0
            if depth is not None:
                dh, dw = depth.shape[:2]
                scaled = np.asarray(contour, np.float32)
                scaled[:, 0, 0] *= dw / width
                scaled[:, 0, 1] *= dh / height
                depth_mask = np.zeros((dh, dw), np.uint8)
                cv2.drawContours(depth_mask, [scaled.astype(np.int32)], -1, 255, -1)
                values = depth[(depth_mask > 0) & np.isfinite(depth) & (depth > 0)]
                if values.size:
                    measured_depth = float(np.median(values))
            if not config['depth_min'] <= measured_depth <= config['depth_max']:
                continue
            score = aspect / (1.0 + abs(math.log(max(area, 1.0) / 400.0)))
            candidates.append({'u': float(cx), 'v': float(cy), 'depth': measured_depth,
                               'area': area, 'aspect': float(aspect), 'angle': float(angle),
                               'score': float(score), 'contour': contour})
        candidates.sort(key=lambda item: item['score'], reverse=True)
        best = candidates[0] if candidates else None
        if best is not None and self.handle_previous_center is not None:
            distance = math.hypot(best['u']-self.handle_previous_center[0],
                                  best['v']-self.handle_previous_center[1])
            stable_count = self.handle_stable_count + 1 if distance <= config['jitter_px'] else 1
        else:
            stable_count = 1 if best is not None else 0
        previous = (best['u'], best['v']) if best is not None else None
        annotated = frame.copy()
        if len(direct_points) >= 3:
            polygon = cv2.convexHull(np.asarray(direct_points, np.float32)).astype(np.int32)
            cv2.polylines(annotated, [polygon], True, (40, 210, 255), 2, cv2.LINE_AA)
            x, y = polygon.reshape(-1, 2).min(axis=0)
            cv2.putText(annotated, 'JOINT CAL AREA (+60px)', (int(x), max(18, int(y)-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (40, 210, 255), 2, cv2.LINE_AA)
        for index, item in enumerate(candidates[:8]):
            color = (40, 255, 80) if index == 0 else (255, 190, 40)
            cv2.drawContours(annotated, [item['contour']], -1, color, 2)
            cv2.drawMarker(annotated, (round(item['u']), round(item['v'])), color,
                           cv2.MARKER_CROSS, 18, 2)
            cv2.putText(annotated, f"H{index+1} {item['depth']:.0f}mm",
                        (round(item['u'])+7, round(item['v'])-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        if locked_target is not None:
            point = (round(locked_target[0]), round(locked_target[1]))
            cv2.drawMarker(annotated, point, (255, 40, 220), cv2.MARKER_TILTED_CROSS, 30, 3)
            cv2.circle(annotated, point, 22, (255, 40, 220), 2)
            cv2.putText(annotated, 'LOCKED TARGET', (point[0]+12, point[1]+25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 40, 220), 2, cv2.LINE_AA)
        if calibration_selected is not None:
            point = (round(calibration_selected[0]), round(calibration_selected[1]))
            cv2.drawMarker(annotated, point, (20, 255, 255), cv2.MARKER_CROSS, 28, 3)
            cv2.putText(annotated, 'CAL POINT', (point[0]+12, point[1]-12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 255, 255), 2, cv2.LINE_AA)
        annotated_jpeg = cv2.imencode('.jpg', cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR),
                                      [cv2.IMWRITE_JPEG_QUALITY, 75])[1]
        mask_jpeg = cv2.imencode('.jpg', mask, [cv2.IMWRITE_JPEG_QUALITY, 85])[1]
        cleaned = [{key: value for key, value in item.items() if key != 'contour'}
                   for item in candidates]
        with self.lock:
            self.handle_detections = cleaned
            self.handle_best = cleaned[0] if cleaned else None
            self.handle_stable_count = stable_count
            self.handle_previous_center = previous
            self.handle_jpeg = bytes(annotated_jpeg)
            self.handle_mask_jpeg = bytes(mask_jpeg)

    def configure_handle_filter(self, data):
        config = {
            'vmax': max(20, min(180, int(data['vmax']))),
            'area_min': max(10, min(5000, int(data['area_min']))),
            'area_max': max(100, min(30000, int(data['area_max']))),
            'aspect_min': max(1.0, min(10.0, float(data['aspect_min']))),
            'depth_min': max(100, min(3000, int(data['depth_min']))),
            'depth_max': max(100, min(4000, int(data['depth_max']))),
            'stable_frames': max(1, min(20, int(data['stable_frames']))),
            'jitter_px': max(1, min(80, int(data['jitter_px']))),
        }
        if config['area_min'] >= config['area_max']:
            raise ValueError('최소 면적은 최대 면적보다 작아야 합니다.')
        if config['depth_min'] >= config['depth_max']:
            raise ValueError('최소 Depth는 최대 Depth보다 작아야 합니다.')
        with self.lock:
            self.handle_filter = config
            self.handle_filter_enabled = True
            self.handle_stable_count = 0
            self.handle_previous_center = None
            self.status = '파란 손잡이 필터 설정을 적용했습니다.'

    def set_handle_filter_enabled(self, enabled):
        with self.lock:
            self.handle_filter_enabled = bool(enabled)
            self.handle_detections = []
            self.handle_best = None
            self.handle_stable_count = 0
            self.handle_previous_center = None
            if not enabled:
                self.test_target_pixel = None
            self.status = ('파란 손잡이 필터를 적용했습니다.' if enabled
                           else '손잡이 필터를 해제했습니다. 자동 목표와 이동을 차단합니다.')

    def handle_state(self):
        with self.lock:
            best = dict(self.handle_best) if self.handle_best is not None else None
            config = dict(self.handle_filter)
            filter_enabled = self.handle_filter_enabled
            stable_count = self.handle_stable_count
            matrix, _ = self.planar_model()
            calibration_error = self.planar_calibration_error()
            target = list(self.test_target_pixel) if self.test_target_pixel is not None else None
            selected = list(self.selected) if self.selected is not None else None
            active_layer = self.active_layer
            fixed_z = float(self.fixed_pick_zs[active_layer])
            direct_samples = self.direct_complete(active_layer)
            joint_complete = len(direct_samples)
            joint_depths = [float(item['depth']) for item in direct_samples]
            joint_depth_min = min(joint_depths) if joint_depths else None
            joint_depth_max = max(joint_depths) if joint_depths else None
            joint_partial = sum(1 for item in self.samples
                                if item.get('direct_layer') == active_layer
                                and item.get('direct_above') is not None
                                and item.get('direct_down') is None)
            drop_sample = self.direct_drop_sample()
            drop_above = drop_sample is not None and drop_sample.get('drop_above') is not None
            drop_down = drop_sample is not None and drop_sample.get('drop_down') is not None
            camera_xyz = self.camera_xyz(
                [best['u'], best['v'], best['depth']] if best is not None else None)
            robot_xyz = None
            if best is not None and matrix is not None:
                xy = cv2.perspectiveTransform(
                    np.asarray([[[best['u'], best['v']]]], np.float32), matrix)[0, 0]
                robot_xyz = [float(xy[0]), float(xy[1]), self.fixed_pick_zs[self.active_layer]]
            return {'best': best, 'count': len(self.handle_detections),
                    'camera_xyz': camera_xyz, 'robot_xyz': robot_xyz,
                    'stable_count': stable_count,
                    'stable': best is not None and stable_count >= config['stable_frames'],
                    'config': config, 'filter_enabled': filter_enabled, 'status': self.status,
                    'calibrated': calibration_error is None,
                    'calibration_error': calibration_error,
                    'target_pixel': target, 'selected': selected, 'active_layer': active_layer,
                    'fixed_z': fixed_z, 'joint_complete': joint_complete,
                    'joint_partial': joint_partial, 'joint_ready': joint_complete >= 3,
                    'joint_depth_min': joint_depth_min, 'joint_depth_max': joint_depth_max,
                    'drop_above': drop_above, 'drop_down': drop_down,
                    'drop_ready': drop_above and drop_down, 'playing': self.playing}

    def set_handle_target(self):
        with self.lock:
            if not self.handle_filter_enabled:
                raise ValueError('손잡이 필터가 OFF입니다. 필터를 적용하세요.')
            best = dict(self.handle_best) if self.handle_best is not None else None
            if best is None:
                raise ValueError('조건을 통과한 손잡이가 없습니다.')
            if self.handle_stable_count < self.handle_filter['stable_frames']:
                raise ValueError('손잡이 위치가 아직 안정되지 않았습니다.')
            self.selected = (best['u'], best['v'], best['depth'])
            self.test_target_pixel = tuple(map(float, self.selected))
            ready = len(self.direct_complete()) >= 3
            self.status = ('최적 손잡이를 목표로 고정했습니다. ' +
                           ('관절 직접 보간으로 이동할 수 있습니다.' if ready
                            else '현재 층 관절 보정 3쌍 이상이 필요합니다.'))

    def select_handle_target(self, u, v):
        """Lock the filtered handle nearest to a click in the overlay."""
        with self.lock:
            if not self.handle_filter_enabled:
                raise ValueError('손잡이 필터가 OFF입니다. 필터를 적용하세요.')
            if not self.handle_detections:
                raise ValueError('조건을 통과한 손잡이가 없습니다.')
            nearest = min(self.handle_detections,
                          key=lambda item: math.hypot(item['u']-u, item['v']-v))
            distance = math.hypot(nearest['u']-u, nearest['v']-v)
            if distance > 80.0:
                raise ValueError('클릭 근처 80px 안에 검출된 손잡이가 없습니다.')
            self.selected = (nearest['u'], nearest['v'], nearest['depth'])
            self.test_target_pixel = tuple(map(float, self.selected))
            ready = len(self.direct_complete()) >= 3
            self.status = (f'클릭한 H 후보를 목표로 고정했습니다 (거리 {distance:.0f}px). ' +
                           ('관절 직접 보간으로 이동할 수 있습니다.' if ready
                            else '현재 층 관절 보정 3쌍 이상이 필요합니다.'))

    def on_rgb(self, msg):
        now = time.monotonic()
        if now - self.last_rgb_time < 0.18:
            return
        self.last_rgb_time = now
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            channels = max(1, msg.step // max(1, msg.width))
            frame = raw.reshape(msg.height, msg.step)[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
            if msg.encoding.lower() in ('bgr8', 'bgra8'):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR if channels == 4 else cv2.COLOR_BGR2RGB)
            elif channels == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
            elif channels == 1:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            self.update_handle_detection(frame)
            shown = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
            with self.lock:
                samples = list(self.samples)
                selected, pick, drop = self.selected, self.pick, self.drop
                planar_points = list(self.planar_points)
                test_target = self.test_target_pixel
            self._draw_workspace(shown, samples, selected, pick, drop, planar_points, test_target)
            # OpenCV JPEG encoder expects BGR.
            encoded = cv2.imencode('.jpg', cv2.cvtColor(shown, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 70])[1]
            with self.lock:
                self.rgb = frame.copy()
                self.jpeg = bytes(encoded)
                self.last_frame_received = time.monotonic()
        except (ValueError, cv2.error):
            pass

    @staticmethod
    def _draw_workspace(frame, samples, selected, pick, drop, planar_points=(), test_target=None):
        """Draw only current clicked/pick/drop targets; hide legacy regions."""
        height, width = frame.shape[:2]
        if len(planar_points) >= 3:
            points = np.asarray([[round(float(p['u']) * width / 640),
                                  round(float(p['v']) * height / 480)]
                                 for p in planar_points], np.int32)
            hull = cv2.convexHull(points)
            cv2.polylines(frame, [hull], True, (40, 220, 255), 2)
            for index, (x, y) in enumerate(points, 1):
                cv2.circle(frame, (int(x), int(y)), 4, (40, 220, 255), -1)
                cv2.putText(frame, f'C{index}', (int(x)+5, int(y)-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40, 220, 255), 1, cv2.LINE_AA)
        if test_target is not None:
            tx = round(float(test_target[0]) * width / 640)
            ty = round(float(test_target[1]) * height / 480)
            cv2.drawMarker(frame, (tx, ty), (60, 255, 80), cv2.MARKER_TILTED_CROSS, 30, 3)
            cv2.putText(frame, 'MOVE TARGET', (tx + 10, ty - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (60, 255, 80), 2, cv2.LINE_AA)
        for point, color, label in ((selected, (255, 255, 255), 'SELECTED'),
                                    (pick, (255, 70, 70), 'PICK'),
                                    (drop, (255, 170, 30), 'DROP')):
            if point is None:
                continue
            x = round(float(point[0]) * width / 640)
            y = round(float(point[1]) * height / 480)
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_CROSS, 26, 2)
            cv2.putText(frame, label, (x + 10, y + 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2, cv2.LINE_AA)
        return
        height, width = frame.shape[:2]
        for mode, color, title in (
            ('pick', (40, 220, 80), 'PICK AREA'),
            ('drop', (60, 150, 255), 'DROP AREA'),
        ):
            complete = [s for s in samples
                        if s.get(mode + '_above') is not None and s.get(mode + '_down') is not None]
            if mode == 'drop':
                preferred = [s for s in complete if s.get('label') in ('DROP', '드롭 위치')]
                if preferred:
                    complete = preferred[:1]
            points = np.array([[round(float(s['u']) * width / 640),
                                round(float(s['v']) * height / 480)] for s in complete], np.int32)
            if len(points) >= 3:
                hull = cv2.convexHull(points)
                cv2.polylines(frame, [hull], True, color, 2)
                x, y, w, h = cv2.boundingRect(hull)
                cv2.putText(frame, title, (x + 5, max(18, y - 7)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
            elif len(points) == 1:
                x, y = points[0]
                cv2.circle(frame, (x, y), 30, color, 2)
                cv2.putText(frame, title + ' (30px)', (max(5, x - 80), max(18, y - 36)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            for point, sample in zip(points, complete):
                x, y = int(point[0]), int(point[1])
                cv2.circle(frame, (x, y), 6, color, -1)
                label = str(sample.get('label', 'POINT'))
                if mode == 'drop':
                    label = 'DROP-1'
                else:
                    label = {'1행 1열': 'P1-1', '1행 2열': 'P1-2',
                             '2행 1열': 'P2-1', '2행 2열': 'P2-2'}.get(label, 'POINT')
                cv2.putText(frame, label, (x + 8, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

        for point, color, label in ((selected, (255, 255, 255), 'SELECTED'),
                                    (pick, (255, 70, 70), 'PICK'),
                                    (drop, (255, 170, 30), 'DROP')):
            if point is None:
                continue
            x = round(float(point[0]) * width / 640)
            y = round(float(point[1]) * height / 480)
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_CROSS, 22, 2)
            cv2.putText(frame, label, (x + 10, y + 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2, cv2.LINE_AA)

    def on_depth(self, msg):
        now = time.monotonic()
        if now - self.last_depth_time < 0.1:
            return
        self.last_depth_time = now
        try:
            if msg.encoding.upper() in ('16UC1', 'MONO16'):
                depth = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
            elif msg.encoding.upper() == '32FC1':
                depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width] * 1000
            else:
                return
            with self.lock:
                self.depth = depth.copy()
        except ValueError:
            pass

    def on_angles(self, msg):
        if len(msg.data) >= 6:
            with self.lock:
                self.angles = [float(v) if v >= 0 else None for v in msg.data[:6]]

    def select(self, u, v):
        with self.lock:
            if self.rgb is None:
                raise ValueError('카메라 영상이 아직 없습니다.')
            h, w = self.rgb.shape[:2]
            u = max(0, min(w - 1, float(u))); v = max(0, min(h - 1, float(v)))
            z = 0.0
            if self.depth is not None:
                dh, dw = self.depth.shape[:2]; du = min(dw - 1, round(u * dw / w)); dv = min(dh - 1, round(v * dh / h))
                used_radius = None
                for radius in (2, 5, 10, 20):
                    p = self.depth[max(0, dv-radius):min(dh, dv+radius+1),
                                   max(0, du-radius):min(dw, du+radius+1)]
                    valid = p[np.isfinite(p) & (p > 0)]
                    if valid.size >= 3:
                        z = float(np.median(valid)); used_radius = radius; break
            self.selected = (u, v, z)
            self.status = ('카메라 좌표를 선택했습니다.' if z > 0
                           else '선택점 주변 20px에도 유효한 Depth가 없습니다.')

    def record_ik_point(self):
        with self.lock:
            if self.selected is None:
                raise ValueError('먼저 영상에서 기준점을 클릭하세요.')
            if any(v is None for v in self.angles):
                raise ValueError('현재 관절값 6개를 모두 읽지 못했습니다.')
            u, v, z = self.selected
            self.ik_points.append({'u': float(u), 'v': float(v), 'depth': float(z),
                                   'angles': [float(x) for x in self.angles]})
            self.save_ik_points()
            self.status = f'IK 보정점 {len(self.ik_points)}개를 기록했습니다.'

    def save_ik_points(self):
        path = Path('/home/intelions/ros2_ws/config/ik_calibration.json')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({'points': self.ik_points}, ensure_ascii=False, indent=2) + '\n')

    def load_ik_points(self):
        try:
            path = Path('/home/intelions/ros2_ws/config/ik_calibration.json')
            self.ik_points = list(json.loads(path.read_text()).get('points', []))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            self.ik_points = []

    def detect(self, kind):
        with self.lock:
            if self.rgb is None:
                raise ValueError('카메라 영상이 아직 없습니다.')
            hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
            if kind == 'drop':
                mask = cv2.inRange(hsv, np.array([35, 70, 45], np.uint8), np.array([95, 255, 255], np.uint8))
                label = '녹색 드롭점'
            elif kind == 'handle':
                # 검은색은 낮은 밝기·낮은 채도로 검출하고 작은 잡음은 제거한다.
                mask = cv2.inRange(hsv, np.array([0, 0, 0], np.uint8), np.array([180, 130, 78], np.uint8))
                label = '검은 손잡이'
            else:
                raise ValueError('알 수 없는 자동 검출 대상입니다.')
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            h, w = self.rgb.shape[:2]
            candidates = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if 20 <= area <= (w * h * 0.35):
                    m = cv2.moments(contour)
                    if m['m00']:
                        candidates.append((area, m['m10'] / m['m00'], m['m01'] / m['m00']))
            if not candidates:
                raise ValueError(f'{label}를 찾지 못했습니다. 조명이나 색상 범위를 확인하세요.')
            # 손잡이는 너무 큰 영역보다 적당한 크기의 후보를 우선한다.
            area, u, v = min(candidates, key=lambda x: abs(math.log(max(x[0], 1)) - math.log(350)))
            self.select(u, v)
            if kind == 'drop':
                self.drop = self.selected
            self.status = f'{label} 자동 검출: u={u:.0f}, v={v:.0f}'

    def record(self, kind, label='기준점', mode='pick'):
        with self.lock:
            if self.selected is None: raise ValueError('먼저 영상을 클릭하세요.')
            if any(v is None for v in self.angles): raise ValueError('현재 관절값 6개를 모두 읽지 못했습니다.')
            pose = list(map(float, self.angles)); u, v, z = self.selected
            if mode not in ('pick', 'drop'):
                raise ValueError('픽 또는 드롭 보정 대상을 선택하세요.')
            above_key, down_key = mode + '_above', mode + '_down'
            mode_name = '픽' if mode == 'pick' else '드롭'
            if kind == 'above':
                sample = next((s for s in self.samples if s.get('label') == label), None)
                if sample is None:
                    sample = {'label': label}
                    self.samples.append(sample)
                sample.update({'u': u, 'v': v, 'depth': z, above_key: pose, down_key: None})
                self.status = f'{label} {mode_name}: 손잡이 위 자세를 기록했습니다.'
            elif kind == 'down':
                sample = next((s for s in self.samples
                               if s.get('label') == label and s.get(above_key) is not None and s.get(down_key) is None), None)
                if sample is None: raise ValueError(f'{label}의 손잡이 위 자세를 먼저 기록하세요.')
                sample[down_key] = pose
                self.status = f"{sample.get('label','기준점')} {mode_name}: 손잡이 잡는 자세를 기록했습니다."
            else: raise ValueError('잘못된 보정 종류입니다.')
            self.save()

    def complete(self, mode='pick'):
        complete = [s for s in self.samples
                    if s.get(mode + '_above') is not None and s.get(mode + '_down') is not None]
        # 드롭은 고정 위치 1곳만 사용한다. 이전 UI에서 2행 2열로
        # 잘못 저장된 드롭 샘플이 있어도 새 DROP 샘플을 우선한다.
        if mode == 'drop':
            preferred = [s for s in complete if s.get('label') in ('DROP', '드롭 위치')]
            if preferred:
                return preferred[:1]
        return complete
    def inside(self, point, mode='pick'):
        c = self.complete(mode)
        if mode == 'drop' and len(c) == 1:
            return math.hypot(c[0]['u'] - point[0], c[0]['v'] - point[1]) <= 30.0
        if len(c) < 3: return False
        hull = cv2.convexHull(np.array([[s['u'], s['v']] for s in c], np.float32))
        # Allow a few pixels of click/depth display rounding at the calibrated
        # boundary, while still rejecting points clearly outside the workspace.
        return cv2.pointPolygonTest(hull, (float(point[0]), float(point[1])), True) >= -60.0
    def interpolate(self, point, kind, mode='pick'):
        result = [0.0]*6; total = 0.0
        for s in self.complete(mode):
            weight = 1.0 / max(4.0, math.hypot(s['u']-point[0], s['v']-point[1]))**2
            for i, value in enumerate(s[mode + '_' + kind]): result[i] += weight*float(value)
            total += weight
        return [v/total for v in result]

    def direct_complete(self, layer=None):
        layer = self.active_layer if layer is None else int(layer)
        result = []
        for sample in self.samples:
            above, down = sample.get('direct_above'), sample.get('direct_down')
            if sample.get('direct_layer') != layer or above is None or down is None:
                continue
            if float(sample.get('depth', 0)) <= 0:
                continue
            # J6 is the gripper. A pick pair must contain an actual J1-J5 arm motion.
            if max(abs(float(down[i]) - float(above[i])) for i in range(5)) < 3.0:
                continue
            result.append(sample)
        return result

    def record_direct_joint(self, kind, label):
        if kind not in ('above', 'down'):
            raise ValueError('위 자세 또는 집기 자세만 기록할 수 있습니다.')
        with self.lock:
            if self.selected is None:
                raise ValueError('먼저 손잡이 영상에서 목표를 클릭하세요.')
            if float(self.selected[2]) <= 0:
                raise ValueError('선택점의 Depth가 0입니다. 유효한 거리값이 나오는 위치를 선택하세요.')
            if any(value is None for value in self.angles):
                raise ValueError('현재 관절값 6개를 모두 읽지 못했습니다.')
            layer = self.active_layer
            sample = next((item for item in self.samples
                           if item.get('direct_layer') == layer and item.get('label') == label), None)
            if kind == 'above':
                if sample is None:
                    sample = {'label': label, 'direct_layer': layer}
                    self.samples.append(sample)
                sample.update({'u': float(self.selected[0]), 'v': float(self.selected[1]),
                               'depth': float(self.selected[2]),
                               'direct_above': list(map(float, self.angles)),
                               'direct_down': None})
            else:
                if sample is None or sample.get('direct_above') is None:
                    raise ValueError(f'{layer}층 {label}의 위 자세를 먼저 기록하세요.')
                down_pose = list(map(float, self.angles))
                if max(abs(down_pose[i] - float(sample['direct_above'][i])) for i in range(5)) < 3.0:
                    raise ValueError('집기 자세에서 J1~J5가 거의 움직이지 않았습니다. J6 그리퍼가 아니라 팔 끝을 실제로 하강시킨 뒤 기록하세요.')
                sample['direct_down'] = down_pose
            self.save()
            self.status = f'{layer}층 {label} {"위" if kind == "above" else "집기"} 자세를 기록했습니다.'

    def delete_direct_joint_labels(self, labels, layer=None):
        layer = self.active_layer if layer is None else int(layer)
        wanted = {str(label) for label in labels}
        if not wanted:
            raise ValueError('삭제할 보정점 이름이 없습니다.')
        with self.lock:
            before = len(self.samples)
            self.samples = [item for item in self.samples
                            if not (item.get('direct_layer') == layer
                                    and str(item.get('label')) in wanted)]
            deleted = before - len(self.samples)
            if deleted == 0:
                raise ValueError(f'{layer}층에서 해당 보정점을 찾지 못했습니다.')
            self.save()
            self.status = f'{layer}층 보정점 {", ".join(sorted(wanted))} 중 {deleted}개를 삭제했습니다.'

    def direct_inside(self, point, layer=None):
        complete = self.direct_complete(layer)
        if len(complete) < 3:
            return False
        if len(point) < 3 or float(point[2]) <= 0:
            return False
        hull = cv2.convexHull(np.asarray([[s['u'], s['v']] for s in complete], np.float32))
        if cv2.pointPolygonTest(hull, (float(point[0]), float(point[1])), True) < -60.0:
            return False
        depths = [float(sample['depth']) for sample in complete]
        if not min(depths) - 120.0 <= float(point[2]) <= max(depths) + 120.0:
            return False
        return min(self.direct_distance(point, sample) for sample in complete) <= 1.8

    @staticmethod
    def direct_distance(point, sample):
        """Normalized 3D distance: 100 image pixels ~= 150 mm of depth."""
        du = (float(point[0]) - float(sample['u'])) / 100.0
        dv = (float(point[1]) - float(sample['v'])) / 100.0
        dz = (float(point[2]) - float(sample['depth'])) / 150.0
        return math.sqrt(du * du + dv * dv + dz * dz)

    def interpolate_direct(self, point, kind, layer=None):
        result, total = [0.0] * 6, 0.0
        neighbors = sorted(self.direct_complete(layer),
                           key=lambda sample: self.direct_distance(point, sample))[:6]
        for sample in neighbors:
            weight = 1.0 / max(0.05, self.direct_distance(point, sample))**2
            for index, value in enumerate(sample['direct_' + kind]):
                result[index] += weight * float(value)
            total += weight
        return [value / total for value in result]

    def direct_drop_sample(self):
        return next((item for item in self.samples if item.get('label') == 'DIRECT_DROP'), None)

    def record_direct_drop(self, kind):
        if kind not in ('above', 'down'):
            raise ValueError('드롭 위 또는 놓기 자세만 기록할 수 있습니다.')
        with self.lock:
            if any(value is None for value in self.angles):
                raise ValueError('현재 관절값 6개를 모두 읽지 못했습니다.')
            sample = self.direct_drop_sample()
            if sample is None:
                sample = {'label': 'DIRECT_DROP'}
                self.samples.append(sample)
            if kind == 'down' and sample.get('drop_above') is None:
                raise ValueError('드롭 위 자세를 먼저 기록하세요.')
            sample['drop_' + kind] = list(map(float, self.angles))
            self.save()
            self.status = f'드롭 {"위" if kind == "above" else "놓기"} 자세를 기록했습니다.'

    @staticmethod
    def _driver_safe_pose(pose):
        minimums = (0.0, 0.0, 0.0, 0.0, 12.0, 22.0)
        maximums = (179.0, 179.0, 180.0, 180.0, 197.0, 180.0)
        return len(pose) == 6 and all(minimums[i] <= pose[i] <= maximums[i]
                                      for i in range(6))

    def start_direct_pick_drop(self, seconds, drop_seconds, opened, closed):
        seconds = max(1.0, min(10.0, float(seconds)))
        drop_seconds = max(3.0, min(15.0, float(drop_seconds)))
        opened = max(22.0, min(180.0, float(opened)))
        closed = max(22.0, min(180.0, float(closed)))
        with self.lock:
            if self.playing:
                raise ValueError('이미 연속 동작 중입니다.')
            if len(self.direct_complete()) < 3:
                raise ValueError('현재 층 관절 보정이 최소 3쌍 필요합니다.')
            if self.test_target_pixel is None:
                raise ValueError('먼저 손잡이 목표를 클릭해 고정하세요.')
            target = tuple(self.test_target_pixel)
            if not self.direct_inside(target):
                raise ValueError('목표의 화면 위치 또는 Depth가 3D 관절 보정 범위 밖입니다.')
            drop = self.direct_drop_sample()
            if drop is None or drop.get('drop_above') is None or drop.get('drop_down') is None:
                raise ValueError('드롭 위 자세와 놓기 자세를 모두 기록하세요.')
            pick_above = self.interpolate_direct(target, 'above')
            pick_down = self.interpolate_direct(target, 'down')
            drop_above = list(map(float, drop['drop_above']))
            drop_down = list(map(float, drop['drop_down']))
            pick_above[5] = opened; pick_down[5] = opened
            close_pose = pick_down.copy(); close_pose[5] = closed
            lift_pose = pick_above.copy(); lift_pose[5] = closed
            drop_above[5] = closed; drop_down[5] = closed
            release_pose = drop_down.copy(); release_pose[5] = opened
            retreat_pose = drop_above.copy(); retreat_pose[5] = opened
            sequence = [pick_above, pick_down, close_pose, lift_pose,
                        drop_above, drop_down, release_pose, retreat_pose]
            durations = [seconds, seconds, 1.0, seconds,
                         drop_seconds, seconds, 1.0, seconds]
            if not all(self._driver_safe_pose(pose) for pose in sequence):
                raise ValueError('연속 동작 자세가 드라이버 안전 관절 범위를 벗어났습니다.')
            self.playing = True
            self.stop_event.clear()
            self.status = '관절 직접 보간 픽→드롭 연속 동작을 시작합니다.'
        self.torque_pub.publish(Bool(data=True))
        threading.Thread(target=self.run_worker_timed,
                         args=(sequence, durations), daemon=True).start()

    def move_direct_joint_stage(self, stage, seconds, gripper):
        """Interpolate recorded pixel-to-joint pose pairs without FK or IK."""
        if stage not in ('above', 'down', 'lift'):
            raise ValueError('알 수 없는 이동 단계입니다.')
        seconds = max(1.0, min(10.0, float(seconds)))
        gripper = max(22.0, min(180.0, float(gripper)))
        with self.lock:
            if len(self.direct_complete()) < 3:
                raise ValueError('화면→관절 보정 자세가 최소 3쌍 필요합니다. 6쌍 이상 권장합니다.')
            if self.test_target_pixel is None:
                raise ValueError('먼저 손잡이 목표를 클릭해 고정하세요.')
            target = tuple(self.test_target_pixel)
            if not self.direct_inside(target):
                raise ValueError('목표의 화면 위치 또는 Depth가 3D 관절 보정 범위 밖입니다.')
            pose = self.interpolate_direct(target, 'down' if stage == 'down' else 'above')
            # Never reuse an invalid J6 reading: the driver rejects the entire pose
            # when J6 is below its calibrated 22-degree safety minimum.
            pose[5] = gripper
            if any(value < 0 or value > (270 if index == 4 else 180)
                   for index, value in enumerate(pose)):
                raise ValueError('보간된 관절 자세가 관절 범위를 벗어났습니다.')
        self.torque_pub.publish(Bool(data=True))
        self.move_pub.publish(Float32MultiArray(data=[*pose, float(round(seconds * 1000))]))
        with self.lock:
            label = '집기 자세' if stage == 'down' else '목표 위 자세'
            self.status = f'{label}로 관절 직접 보간 이동 명령을 보냈습니다.'

    def _ik_pose(self, point, height_offset_cm, gripper):
        """Convert a camera point to robot XYZ via recorded camera/FK pairs, then solve IK."""
        if len(self.ik_points) < 4:
            raise ValueError('IK 보정점이 4개 이상 필요합니다.')
        rows = []
        robot = []
        for item in self.ik_points:
            angles = item.get('angles', [])
            if len(angles) < 5 or item.get('depth', 0) <= 0:
                continue
            xyz_cm, _ = forward_kinematics_servo(angles[:5])
            rows.append([float(item['u']), float(item['v']), float(item['depth']), 1.0])
            robot.append(list(xyz_cm))
        if len(rows) < 4:
            raise ValueError('유효한 IK 보정점이 4개 이상 필요합니다.')
        affine, _, _, _ = np.linalg.lstsq(np.asarray(rows), np.asarray(robot), rcond=None)
        z = float(point[2])
        if z <= 0:
            z = min(self.ik_points, key=lambda p: math.hypot(p['u']-point[0], p['v']-point[1])).get('depth', 0)
        camera_row = np.array([float(point[0]), float(point[1]), z, 1.0])
        target_cm = camera_row @ affine
        target_cm[2] += float(height_offset_cm)
        nearest = min(self.ik_points, key=lambda p: math.hypot(p['u']-point[0], p['v']-point[1]))
        _, rpy_deg = forward_kinematics_servo(nearest['angles'][:5])
        # DOFBOT은 5-DOF이므로 픽 작업에서는 6축 자세 일치보다 위치를
        # 우선한다. 회전 오차는 고정 그리퍼 자세로 허용한다.
        solved, ok = None, False
        # 특정 보정점의 자세를 초기값으로 쓰면 수치해석이 관절 한계에
        # 걸릴 수 있으므로 여러 초기 자세에서 재시도한다.
        seeds = [nearest['angles'][:5], [90.0, 30.0, 90.0, 0.0, 99.0],
                 [90.0, 0.0, 90.0, 0.0, 99.0], self.angles[:5]]
        for seed in seeds:
            if any(v is None for v in seed):
                continue
            solved, ok = inverse_kinematics_servo(
                target_cm.tolist(), rpy_deg,
                q_init_servo=seed,
                rot_weight=0.0, tol_rot=10.0, max_iter=1000)
            if ok:
                break
        if not ok:
            raise ValueError(f'IK 계산 실패 (목표 XYZ={target_cm.round(1).tolist()} cm)')
        pose = [float(v) for v in solved] + [float(gripper)]
        if any(v < 0 or v > (270 if i == 4 else 180) for i, v in enumerate(pose)):
            raise ValueError('IK 결과가 관절 제한을 벗어났습니다.')
        return pose

    def start_run(self, seconds, opened, closed):
        with self.lock:
            if self.playing: raise ValueError('이미 동작 중입니다.')
            if self.pick is None or self.drop is None: raise ValueError('픽과 드롭 좌표를 모두 지정하세요.')
            pu=self._ik_pose(self.pick, 5.0, opened); pd=self._ik_pose(self.pick, 0.0, opened)
            du=self._ik_pose(self.drop, 5.0, closed); dd=self._ik_pose(self.drop, 0.0, closed)
            for pose in (pu,pd,du,dd):
                if any(v<0 or v>(270 if i==4 else 180) for i,v in enumerate(pose)): raise ValueError('보간 자세가 관절 범위를 벗어났습니다.')
            pu[5]=opened; pd[5]=opened; pc=pd.copy(); pc[5]=closed; pl=pu.copy(); pl[5]=closed
            du[5]=closed; dd[5]=closed; rel=dd.copy(); rel[5]=opened; final=du.copy(); final[5]=opened
            sequence=[pu,pd,pc,pl,du,dd,rel,final]
            self.playing=True; self.stop_event.clear(); self.status='Pick & Place 실행을 시작합니다.'
        self.torque_pub.publish(Bool(data=True))
        threading.Thread(target=self.run_worker, args=(sequence, seconds), daemon=True).start()

    def run_worker(self, sequence, seconds):
        for i, pose in enumerate(sequence):
            if self.stop_event.is_set(): break
            self.move_pub.publish(Float32MultiArray(data=[*pose, float(round(seconds*1000))]))
            with self.lock: self.status=f'동작 {i+1}/8 실행 중'
            if self.stop_event.wait(seconds+0.35): break
        with self.lock:
            self.playing=False; self.status='정지했습니다.' if self.stop_event.is_set() else 'Pick & Place 완료'

    def run_worker_timed(self, sequence, durations):
        labels = ('픽 위', '픽 하강', '그리퍼 닫기', '픽 상승',
                  '드롭으로 저속 이동', '드롭 하강', '그리퍼 열기', '드롭 복귀')
        for index, (pose, duration) in enumerate(zip(sequence, durations)):
            if self.stop_event.is_set():
                break
            self.move_pub.publish(Float32MultiArray(
                data=[*pose, float(round(duration * 1000))]))
            with self.lock:
                self.status = f'{labels[index]} {index+1}/8 · {duration:.1f}초'
            if self.stop_event.wait(duration + 0.35):
                break
        with self.lock:
            self.playing = False
            self.status = ('정지했습니다.' if self.stop_event.is_set()
                           else '집기→드롭 연속 동작을 완료했습니다.')

    def stop(self):
        self.stop_event.set()
        with self.lock:
            pose = None if any(v is None for v in self.angles) else list(self.angles)
            self.status='정지 명령을 보냈습니다.'
        if pose: self.move_pub.publish(Float32MultiArray(data=[*pose,100.0]))

    def reconnect_camera(self):
        with self.lock:
            self.rgb = self.depth = self.jpeg = None
            self.last_frame_received = 0.0
            self.status = 'ASCamera 드라이버를 다시 시작하는 중입니다…'
        threading.Thread(target=self._restart_camera_worker, daemon=True).start()

    def capture_pair(self):
        """Save clean paired RGB frames plus depth and acquisition metadata."""
        now = time.monotonic()
        with self.lock:
            if self.rgb is None or now - self.last_frame_received >= 3.0:
                raise ValueError('RGB-D 카메라 영상이 없습니다.')
            rgb = self.rgb.copy()
            depth = self.depth.copy() if self.depth is not None else None
            angles = list(self.angles)
            rgb_age = now - self.last_frame_received
        with self.usb_camera.lock:
            if self.usb_camera.clean_jpeg is None or now - self.usb_camera.last_frame_received >= 3.0:
                raise ValueError('C270 카메라 영상이 없습니다.')
            usb_jpeg = self.usb_camera.clean_jpeg
            usb_age = now - self.usb_camera.last_frame_received

        for name in ('rgbd', 'depth', 'c270', 'meta'):
            (DATASET_DIR / name).mkdir(parents=True, exist_ok=True)
        self.capture_count += 1
        capture_id = f'capture_{self.capture_count:06d}_{time.strftime("%Y%m%d_%H%M%S")}'
        rgb_path = DATASET_DIR / 'rgbd' / f'{capture_id}.jpg'
        depth_path = DATASET_DIR / 'depth' / f'{capture_id}.png'
        usb_path = DATASET_DIR / 'c270' / f'{capture_id}.jpg'
        meta_path = DATASET_DIR / 'meta' / f'{capture_id}.json'
        if not cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                           [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise ValueError('RGB-D 이미지 저장에 실패했습니다.')
        usb_path.write_bytes(usb_jpeg)
        depth_saved = False
        if depth is not None:
            depth_saved = cv2.imwrite(str(depth_path), np.clip(depth, 0, 65535).astype(np.uint16))
        meta = {
            'id': capture_id,
            'captured_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'rgbd_image': str(rgb_path), 'c270_image': str(usb_path),
            'depth_image': str(depth_path) if depth_saved else None,
            'rgbd_frame_age_seconds': rgb_age, 'c270_frame_age_seconds': usb_age,
            'joint_angles': angles,
        }
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + '\n')
        with self.lock:
            self.status = f'학습 이미지 {self.capture_count}번 세트를 저장했습니다.'
        return capture_id

    def capture_download(self):
        """Build a paired training sample ZIP in memory for browser download."""
        now = time.monotonic()
        with self.lock:
            if self.rgb is None or now - self.last_frame_received >= 3.0:
                raise ValueError('RGB-D 카메라 영상이 없습니다.')
            rgb = self.rgb.copy()
            depth = self.depth.copy() if self.depth is not None else None
            angles = list(self.angles)
            rgb_age = now - self.last_frame_received
        with self.usb_camera.lock:
            if self.usb_camera.clean_jpeg is None or now - self.usb_camera.last_frame_received >= 3.0:
                raise ValueError('C270 카메라 영상이 없습니다.')
            usb_jpeg = bytes(self.usb_camera.clean_jpeg)
            usb_age = now - self.usb_camera.last_frame_received

        ok, rgb_encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                       [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise ValueError('RGB-D 이미지 인코딩에 실패했습니다.')
        depth_bytes = None
        if depth is not None:
            ok, depth_encoded = cv2.imencode('.png', np.clip(depth, 0, 65535).astype(np.uint16))
            if ok:
                depth_bytes = bytes(depth_encoded)
        self.download_count += 1
        capture_id = f'dofbot_capture_{time.strftime("%Y%m%d_%H%M%S")}_{self.download_count:03d}'
        metadata = {
            'id': capture_id,
            'captured_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'rgbd_image': 'rgbd.jpg', 'c270_image': 'c270.jpg',
            'depth_image': 'depth.png' if depth_bytes is not None else None,
            'rgbd_frame_age_seconds': rgb_age, 'c270_frame_age_seconds': usb_age,
            'joint_angles': angles,
        }
        output = io.BytesIO()
        with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
            archive.writestr('rgbd.jpg', bytes(rgb_encoded))
            archive.writestr('c270.jpg', usb_jpeg)
            if depth_bytes is not None:
                archive.writestr('depth.png', depth_bytes)
            archive.writestr('metadata.json', json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
        with self.lock:
            self.status = f'PC용 학습 이미지 {self.download_count}번 ZIP을 생성했습니다.'
        return output.getvalue(), capture_id + '.zip'

    def _restart_camera_worker(self):
        target = '/install/ascamera/lib/ascamera/ascamera_node'
        killed = 0
        for name in os.listdir('/proc'):
            if not name.isdigit():
                continue
            try:
                cmdline = Path('/proc', name, 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='ignore')
                if target in cmdline:
                    os.kill(int(name), signal.SIGTERM)
                    killed += 1
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
        time.sleep(3.0)
        with self.lock:
            self.status = ('카메라 드라이버 재시작 완료. 영상을 기다립니다.' if killed
                           else '카메라 노드를 찾지 못했습니다. 카메라 launch 상태를 확인하세요.')

    def save(self):
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        CALIBRATION_FILE.write_text(json.dumps({'samples':self.samples},ensure_ascii=False,indent=2)+'\n')
    def load(self):
        try:
            self.samples=list(json.loads(CALIBRATION_FILE.read_text()).get('samples',[]))
            # Migrate the previous single-workspace format to the pick side.
            for sample in self.samples:
                if 'above' in sample and 'pick_above' not in sample:
                    sample['pick_above'], sample['pick_down'] = sample.get('above'), sample.get('down')
                    sample.pop('above', None); sample.pop('down', None)
        except (OSError,ValueError,TypeError,json.JSONDecodeError): self.samples=[]
    def state(self):
        with self.lock:
            h,w=(self.rgb.shape[:2] if self.rgb is not None else (0,0))
            age=(time.monotonic()-self.last_frame_received if self.last_frame_received else 0.0)
            state = {'selected':self.selected,'pick':self.pick,'drop':self.drop,'angles':self.angles,'samples':len(self.samples),'pick_complete':len(self.complete('pick')),'drop_complete':len(self.complete('drop')),'ik_points':len(self.ik_points),'capture_count':self.capture_count,'download_count':self.download_count,'status':self.status,'camera':self.rgb is not None and age<3.0,'frame_age':age,'width':w,'height':h,'playing':self.playing}
        ready, usb_age, usb_width, usb_height, usb_error, usb_selected = self.usb_camera.state()
        state.update({'usb_camera':ready,'usb_frame_age':usb_age,'usb_width':usb_width,
                      'usb_height':usb_height,'usb_error':usb_error,'usb_selected':usb_selected})
        return state


class Handler(BaseHTTPRequestHandler):
    controller = None
    def log_message(self, fmt, *args): pass
    def send_bytes(self, data, content_type, status=200):
        self.send_response(status); self.send_header('Content-Type',content_type); self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(data))); self.end_headers(); self.wfile.write(data)
    def json(self, data, status=200): self.send_bytes(json.dumps(data,ensure_ascii=False).encode(),'application/json; charset=utf-8',status)
    def download(self, data, filename):
        self.send_response(200); self.send_header('Content-Type','application/zip'); self.send_header('Cache-Control','no-store'); self.send_header('Content-Disposition',f'attachment; filename="{filename}"'); self.send_header('Content-Length',str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_GET(self):
        path=urlparse(self.path).path
        if path=='/': self.send_bytes(PAGE.encode(),'text/html; charset=utf-8')
        elif path=='/xyz': self.send_bytes(XYZ_PAGE.encode(),'text/html; charset=utf-8')
        elif path=='/handle': self.send_bytes(HANDLE_PAGE.encode(),'text/html; charset=utf-8')
        elif path=='/api/status': self.json(self.controller.state())
        elif path=='/api/xyz/status': self.json(self.controller.planar_state())
        elif path=='/api/handle/status': self.json(self.controller.handle_state())
        elif path=='/stream': self.stream('depth')
        elif path=='/stream/usb': self.stream('usb')
        elif path=='/stream/handle': self.stream('handle')
        elif path=='/stream/handle-mask': self.stream('handle-mask')
        else: self.json({'error':'찾을 수 없습니다.'},404)
    def stream(self, source):
        self.send_response(200); self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame'); self.send_header('Cache-Control','no-store'); self.end_headers()
        last=None
        try:
            while True:
                if source == 'usb':
                    with self.controller.usb_camera.lock: frame=self.controller.usb_camera.jpeg
                elif source == 'handle':
                    with self.controller.lock: frame=self.controller.handle_jpeg
                elif source == 'handle-mask':
                    with self.controller.lock: frame=self.controller.handle_mask_jpeg
                else:
                    with self.controller.lock: frame=self.controller.jpeg
                if frame and frame is not last:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+b'\r\n\r\n'+frame+b'\r\n'); self.wfile.flush(); last=frame
                time.sleep(.18)
        except (BrokenPipeError,ConnectionResetError): pass
    def do_POST(self):
        try:
            size=int(self.headers.get('Content-Length','0')); data=json.loads(self.rfile.read(size) or b'{}'); path=urlparse(self.path).path; c=self.controller
            if path=='/api/capture/download':
                archive, filename = c.capture_download(); self.download(archive, filename); return
            if path=='/api/select': c.select(float(data['u']),float(data['v']))
            elif path=='/api/select/usb':
                point = c.usb_camera.select(float(data['u']), float(data['v']))
                c.status = f'C270 좌표를 선택했습니다: u={point[0]:.0f}, v={point[1]:.0f}'
            elif path=='/api/capture': c.capture_pair()
            elif path=='/api/xyz/record': c.record_planar_point()
            elif path=='/api/xyz/target': c.set_planar_target()
            elif path=='/api/xyz/clear':
                with c.lock:
                    c.planar_points = []; c.test_target_pixel = None; c.save_planar_calibration()
                    c.status = 'XY 평면 보정점을 삭제했습니다.'
            elif path=='/api/xyz/fixed-z':
                layer = int(data['layer'])
                if layer not in (1, 2): raise ValueError('층은 1 또는 2여야 합니다.')
                z = float(data['z'])
                if not -10.0 <= z <= 50.0: raise ValueError('고정 Z는 -10~50 cm 범위로 설정하세요.')
                with c.lock:
                    c.fixed_pick_zs[layer] = z; c.save_planar_calibration()
                    c.status = f'{layer}층 고정 픽 Z를 {z:.2f} cm로 저장했습니다.'
            elif path=='/api/xyz/use-current-z':
                layer = int(data['layer'])
                if layer not in (1, 2): raise ValueError('층은 1 또는 2여야 합니다.')
                with c.lock:
                    current = c.current_robot_xyz()
                    if current is None: raise ValueError('현재 로봇 관절값을 읽지 못했습니다.')
                    c.fixed_pick_zs[layer] = float(current[2]); c.save_planar_calibration()
                    c.status = f'현재 로봇 Z {c.fixed_pick_zs[layer]:.2f} cm를 {layer}층으로 저장했습니다.'
                self.json({'ok':True,'status':c.status,'fixed_z':c.fixed_pick_zs[layer]}); return
            elif path=='/api/xyz/layer':
                layer = int(data['layer'])
                if layer not in (1, 2): raise ValueError('층은 1 또는 2여야 합니다.')
                with c.lock:
                    c.active_layer = layer; c.save_planar_calibration()
                    c.status = f'{layer}층 픽 높이를 선택했습니다.'
            elif path=='/api/xyz/move':
                c.move_planar_stage(str(data['stage']), float(data['approach']), float(data['seconds']))
            elif path=='/api/xyz/gripper':
                c.command_gripper(str(data['action']), float(data['open']),
                                  float(data['closed']), float(data.get('seconds', 1.0)))
            elif path=='/api/handle/config': c.configure_handle_filter(data)
            elif path=='/api/handle/filter-enabled': c.set_handle_filter_enabled(bool(data['enabled']))
            elif path=='/api/handle/target': c.set_handle_target()
            elif path=='/api/handle/select': c.select_handle_target(float(data['u']), float(data['v']))
            elif path=='/api/joint/select':
                c.select(float(data['u']), float(data['v']))
                with c.lock: c.status = '관절 보정용 자유 클릭 좌표를 선택했습니다.'
            elif path=='/api/joint/record': c.record_direct_joint(str(data['kind']), str(data.get('label', 'P1')))
            elif path=='/api/joint/clear':
                with c.lock:
                    layer = c.active_layer
                    c.samples = [item for item in c.samples if item.get('direct_layer') != layer]
                    c.save(); c.status = f'{layer}층 화면→관절 직접 보정 기록을 삭제했습니다.'
            elif path=='/api/joint/delete':
                c.delete_direct_joint_labels(data.get('labels', []), data.get('layer'))
            elif path=='/api/joint/move':
                c.move_direct_joint_stage(str(data['stage']), float(data['seconds']),
                                          float(data.get('gripper', 90.0)))
            elif path=='/api/joint/drop/record': c.record_direct_drop(str(data['kind']))
            elif path=='/api/joint/run':
                c.start_direct_pick_drop(float(data['seconds']),
                                         float(data.get('drop_seconds', 6.0)),
                                         float(data['open']), float(data['closed']))
            elif path=='/api/ik/record': c.record_ik_point()
            elif path=='/api/ik/clear':
                with c.lock:
                    c.ik_points = []
                    c.save_ik_points()
                    c.status = 'IK 보정 기록을 삭제했습니다.'
            elif path=='/api/detect': c.detect(str(data.get('kind','')))
            elif path=='/api/target':
                with c.lock:
                    if c.selected is None: raise ValueError('먼저 영상을 클릭하세요.')
                    if data.get('target')=='pick': c.pick=c.selected; c.status='픽 좌표를 지정했습니다.'
                    elif data.get('target')=='drop': c.drop=c.selected; c.status='드롭 좌표를 지정했습니다.'
                    else: raise ValueError('잘못된 대상입니다.')
            elif path=='/api/torque':
                enabled = bool(data.get('enabled', data.get('on', False)))
                c.torque_pub.publish(Bool(data=enabled)); c.status='Torque ON' if enabled else 'Torque OFF'
            elif path=='/api/calibrate': c.record(str(data['kind']),str(data.get('label','기준점')),str(data.get('mode','pick')))
            elif path=='/api/camera/reconnect': c.reconnect_camera()
            elif path=='/api/camera/usb/reconnect':
                c.usb_camera.reconnect(); c.status='C270 카메라를 다시 연결하는 중입니다…'
            elif path=='/api/clear':
                with c.lock: c.samples=[]; c.save(); c.status='보정점을 모두 삭제했습니다.'
            elif path=='/api/run': c.start_run(max(.5,min(10.,float(data['seconds']))),max(0.,min(180.,float(data['open']))),max(0.,min(180.,float(data['closed']))))
            elif path=='/api/stop': c.stop()
            else: return self.json({'error':'찾을 수 없습니다.'},404)
            self.json({'ok':True,'status':c.status})
        except (KeyError,ValueError,TypeError,json.JSONDecodeError) as error: self.json({'error':str(error)},400)


def main():
    rclpy.init(); controller=Controller(); Handler.controller=controller
    server=ThreadingHTTPServer(('0.0.0.0',8080),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    controller.get_logger().info('Web GUI: http://0.0.0.0:8080')
    try: rclpy.spin(controller)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        server.shutdown(); controller.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
