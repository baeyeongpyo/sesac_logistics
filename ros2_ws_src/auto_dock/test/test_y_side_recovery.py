"""Offline recovery, terminal status and unchanged model endpoint checks."""
import ast,math,queue
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from auto_dock import y_place_sequence as seq
from auto_dock.y_place_fast_prediction import endpoint
from auto_dock.y_place_square_motion import base_coefficients
from auto_dock.y_place import frozen_profile,apply_event
from auto_dock.loaded_response_planner import simulate_actions,ResponseState
from auto_dock.auto_dock_node import AutoDockNode

@pytest.mark.parametrize('side,recovered', [('left',True),('right',True),('left',False),('both',False)])
def test_one_small_peek_then_success_or_terminal_failure(side,recovered):
 tree=ast.parse(Path(seq.__file__).read_text())
 run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run_sequence')
 block=next(n for n in run.body[0].body if isinstance(n,ast.If) and 'reacquire_with_backup' in ast.unparse(n.test))
 actions=[];calls=[]
 scope=dict(vars(seq),node=NS(detection_failure={'missing_side':side,'reason':'side_or_interior_x_missing'}),args=frozen_profile()[0],reacquire_with_backup=lambda _:False,execute=lambda a:actions.extend(a),wait_stopped=lambda _:0.,reacquire=lambda label:calls.append(label) or recovered)
 code=compile(ast.Module(body=[block],type_ignores=[]),'recovery','exec')
 if recovered:exec(code,scope)
 else:
  with pytest.raises(RuntimeError,match='y_place_topline_recovery_failed'):exec(code,scope)
 assert len(actions)==(0 if side=='both' else 1)
 if actions:
  action=actions[0];assert action['drive'][:2]==[0.,0.]
  assert action['drive'][2]==(.35 if side=='left' else -.35)
  assert math.degrees(abs(action['drive'][2])*action['duration_sec'])==pytest.approx(2.)
  assert len(calls)==1

@pytest.mark.parametrize('seed',range(10))
def test_fast_endpoint_matches_full_model(seed):
 rng=np.random.default_rng(seed);c=base_coefficients(frozen_profile()[1]);state=ResponseState(yaw_deg=float(rng.uniform(-90,90)),equilibrium_deg=float(rng.uniform(-30,30)),pending_deg=float(rng.uniform(-10,10)),history_known=True)
 actions=[dict(drive=[0.,0.,.35],duration_sec=float(rng.uniform(.05,1.))),dict(drive=[.1,0.,0.],duration_sec=float(rng.uniform(.05,4.))),dict(drive=[0.,0.,-.35],duration_sec=float(rng.uniform(.05,1.))),dict(drive=[0.,0.,0.],duration_sec=.5)]
 np.testing.assert_allclose(endpoint(actions,state,c),simulate_actions(actions,state,c)['path'][-1],atol=1e-11,rtol=0.)

def test_error_result_is_repeated_until_next_mission(monkeypatch):
 statuses=[];host=NS(state='y_slot_centering',publish_status=lambda *a,**k:statuses.append((a,k)))
 host.cancel=lambda _:setattr(host,'state','idle')
 apply_event(host,{'phase':'result','error':'y_place_topline_recovery_failed:right'})
 assert statuses[-1][0][0]=='cancelled' and statuses[-1][1]['terminal'] is True
 monkeypatch.setattr('auto_dock.auto_dock_node.time.monotonic',lambda:host.y_place_failure_sent_at+1.1)
 AutoDockNode.tick_y_place(host)
 assert len(statuses)==2 and statuses[1][0]==statuses[0][0]
 assert statuses[1][1]['error']==host.y_place_failure
