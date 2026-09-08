import ast
from pathlib import Path
import cv2
import numpy as np
from auto_dock.y_place_slot_sides import interior_yellow_evidence
from auto_dock import y_place_square_geometry as geometry


def evidence(frame):
    return interior_yellow_evidence(frame,dict(k=0.,q=20.,point=[20.,20.]),dict(k=0.,q=180.,point=[180.,20.]),100.,100.,20.)


def test_partial_yellow_patch_without_x_is_accepted():
    frame=np.full((200,200,3),220,np.uint8)
    cv2.rectangle(frame,(90,85),(110,105),(0,255,255),-1)
    assert evidence(frame) is not None


def test_border_yellow_does_not_count():
    frame=np.full((200,200,3),220,np.uint8)
    cv2.rectangle(frame,(20,20),(180,180),(0,255,255),12)
    assert evidence(frame) is None


def test_two_outer_yellow_patches_cannot_fill_empty_middle():
    frame=np.full((200,200,3),220,np.uint8)
    for x in [45,155]:cv2.rectangle(frame,(x-10,70),(x+10,130),(0,255,255),-1)
    assert evidence(frame) is None


def test_scattered_noise_is_rejected():
    frame=np.full((200,200,3),220,np.uint8)
    frame[60:140:4,78:122:4]=(0,255,255)
    assert evidence(frame) is None


def test_tracking_cannot_return_without_interior_check():
    tree=ast.parse(Path(geometry.__file__).read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='detect_locked_topline')
    # Execute the real final gate with fake already-tracked sides, no camera/ROS.
    start=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and 'interior_yellow_evidence' in ast.unparse(n))
    check=ast.FunctionDef(name='check',args=ast.arguments(posonlyargs=[],args=[],kwonlyargs=[],kw_defaults=[],defaults=[]),body=fn.body[start:],decorator_list=[])
    code=compile(ast.fix_missing_locations(ast.Module(body=[check],type_ignores=[])),'gate','exec')
    scope=dict(frame=None,sides=[{},{}],cx=0,cy=0,top_y=0,band={},red_only=True,interior_yellow_evidence=lambda *_:None)
    exec(code,scope)
    assert scope['check']() is None
