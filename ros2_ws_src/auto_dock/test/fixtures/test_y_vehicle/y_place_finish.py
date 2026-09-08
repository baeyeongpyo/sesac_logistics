"""Pure fork acknowledgement gate and nominal unloaded retreat plan."""
import json
import math


class ForkDownGate:
    def __init__(self):
        self.active=False
        self.result=None

    def begin(self):
        self.result=None
        self.active=True

    def receive(self,data):
        if not self.active:return
        try:payload=json.loads(data)
        except (TypeError,ValueError):return
        if not isinstance(payload,dict):return
        state=payload.get('state')
        if state in ('DOWN_COMPLETE','FAILED'):
            self.result=payload

    def complete(self):
        if self.result and self.result['state']=='FAILED':
            raise RuntimeError('fork_down_failed:'+str(self.result.get('error','')))
        return bool(self.result and self.result['state']=='DOWN_COMPLETE')


def unloaded_retreat(distance_cm,speed=.1):
    if not all(math.isfinite(v) for v in (distance_cm,speed)) or distance_cm<=0 or speed<.1:
        raise ValueError('invalid_unloaded_retreat')
    return dict(action='reverse_after_fork_down',drive=[-speed,0.,0.],
                duration_sec=distance_cm/(100*speed))


def camera_reacquisition_retreat(speed=.1):
    """Nominal loaded 10cm straight reverse; measure remaining travel visually."""
    if not math.isfinite(speed) or speed<.1:
        raise ValueError('invalid_reacquisition_reverse_speed')
    return dict(action='reverse_to_reacquire_topline',drive=[-speed,0.,0.],
                duration_sec=10./(100*speed))
