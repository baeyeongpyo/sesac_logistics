"""Pure fork acknowledgement and reverse plan tests, no hardware execution."""
import unittest
from y_place_finish import ForkDownGate,unloaded_retreat


class FinishTests(unittest.TestCase):
    def test_only_new_down_complete_allows_reverse(self):
        gate=ForkDownGate()
        gate.receive('{"state":"DOWN_COMPLETE"}')
        gate.begin()
        self.assertFalse(gate.complete())
        for data in ['bad','[]','{"state":"UP_COMPLETE"}']:
            gate.receive(data)
            self.assertFalse(gate.complete())
        gate.receive('{"state":"DOWN_COMPLETE"}')
        self.assertTrue(gate.complete())

    def test_failed_fork_does_not_reverse(self):
        gate=ForkDownGate();gate.begin();gate.receive('{"state":"FAILED","error":"limit"}')
        with self.assertRaises(RuntimeError):gate.complete()

    def test_unloaded_reverse_uses_insertion_distance_and_no_loaded_gain(self):
        action=unloaded_retreat(35)
        self.assertEqual(action['drive'],[-.1,0,0])
        self.assertEqual(action['duration_sec'],3.5)
        with self.assertRaises(ValueError):unloaded_retreat(35,.05)
