import importlib.util
from pathlib import Path
from unittest import TestCase
from unittest.mock import MagicMock, patch


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'gz_pose_to_odom.py'
SPEC = importlib.util.spec_from_file_location('gz_pose_to_odom', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GazeboPoseToOdomMainTest(TestCase):
    def test_keyboard_interrupt_does_not_shutdown_an_already_closed_context(self):
        node = MagicMock()

        with patch.object(MODULE, 'GazeboPoseToOdom', return_value=node), \
             patch.object(MODULE.rclpy, 'init'), \
             patch.object(MODULE.rclpy, 'spin', side_effect=KeyboardInterrupt), \
             patch.object(MODULE.rclpy, 'ok', return_value=False), \
             patch.object(MODULE.rclpy, 'shutdown') as shutdown:
            try:
                MODULE.main()
            except KeyboardInterrupt as error:
                self.fail(f'KeyboardInterrupt must be treated as normal shutdown: {error!r}')

        node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()


if __name__ == '__main__':
    import unittest

    unittest.main()
