import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


class SlamContractTest(unittest.TestCase):
    def test_mapping_frames_and_topic_are_pinned(self):
        text = (PACKAGE / 'config/slam.yaml').read_text()
        for line in (
            'use_sim_time: true',
            'mode: mapping',
            'map_frame: map',
            'odom_frame: robot_1/odom',
            'base_frame: robot_1/base_footprint',
            'scan_topic: /robot_1/scan_raw',
        ):
            self.assertIn(line, text)

    def test_launch_uses_sync_slam_toolbox(self):
        text = (PACKAGE / 'launch/mapping.launch.py').read_text()
        self.assertIn("executable='sync_slam_toolbox_node'", text)
        self.assertIn("name='slam_toolbox'", text)
