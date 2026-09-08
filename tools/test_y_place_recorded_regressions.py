"""Optional regressions on saved pixels only; never open a live camera or ROS."""
import unittest
from pathlib import Path

import cv2
import numpy as np

from y_place_square_geometry import detect_nearest_topline


CORPUS = Path(__file__).resolve().parents[1] / 'analysis/warning_intersections_corpus'


class RecordedRegressions(unittest.TestCase):
    def test_oblique_nearest_slot_failure_does_not_select_right_neighbour(self):
        source = CORPUS / 'teleop_20260905_154711.mp4'
        if not source.exists():
            self.skipTest('Optional recorded corpus is not installed')
        cap = cv2.VideoCapture(str(source))
        try:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 89)
            ok, frame = cap.read()
        finally:
            cap.release()
        self.assertTrue(ok)
        # This visible centre slot still lacks reliable side evidence.
        # Returning the valid right neighbour would move to the wrong slot.
        self.assertIsNone(detect_nearest_topline(frame))

    def test_original_stage_frames_keep_physical_topline_endpoints(self):
        expected = {
            'test_y_20260906_222616_858': [[294,274],[473,283]],
            'test_y_20260906_222345_858': [[360,291],[566,308]],
        }
        sources = {p.parent.name: p for p in CORPUS.rglob('stage_reacquisition_raw.png')}
        if not all(name in sources for name in expected):
            self.skipTest('Optional original stage frames are not installed')
        for name, points in expected.items():
            with self.subTest(name=name):
                result = detect_nearest_topline(cv2.imread(str(sources[name])))
                self.assertIsNotNone(result)
                np.testing.assert_allclose(result['top_edge_px'], points, atol=8)


if __name__ == '__main__':
    unittest.main()
