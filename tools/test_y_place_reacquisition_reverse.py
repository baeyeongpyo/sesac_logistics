"""Offline recovery control-flow tests; never initialize ROS."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from y_place_finish import camera_reacquisition_retreat


class ReverseReacquisitionTests(unittest.TestCase):
    def flow(self,observations):
        tree=ast.parse(Path(__file__).with_name('y_place_square_topline_trial.py').read_text())
        fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='reacquire_with_backup')
        events=[];results=iter(observations)
        def observe(label):events.append(('observe',label));return next(results)
        scope=dict(reacquire=observe,camera_reacquisition_retreat=camera_reacquisition_retreat,
                   args=SimpleNamespace(speed=.1),output=Path('/unused'),write_json=lambda *args:None,
                   execute=lambda actions:events.append(('execute',actions)),
                   wait_stopped=lambda label:events.append(('stopped',label)) or 0.)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'recovery','exec'),scope)
        return scope['reacquire_with_backup']('midpoint_reacquisition'),events

    def test_visible_line_does_not_reverse(self):
        found,events=self.flow([True]);self.assertTrue(found);self.assertEqual(len(events),1)

    def test_missing_line_reverses_stops_then_observes_again(self):
        found,events=self.flow([False,True]);self.assertTrue(found)
        self.assertEqual([e[0] for e in events],['observe','execute','stopped','observe'])
        action=events[1][1][0]
        self.assertEqual(action['drive'],[-.1,0.,0.]);self.assertEqual(action['duration_sec'],1.)
        self.assertTrue(events[-1][1].endswith('_after_reverse'))

    def test_still_missing_returns_failure_without_blind_repeated_reverse(self):
        found,events=self.flow([False,False]);self.assertFalse(found)
        self.assertEqual(sum(e[0]=='execute' for e in events),1)


if __name__=='__main__':unittest.main()
