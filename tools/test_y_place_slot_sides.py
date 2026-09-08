"""Pixel-level side-intersection regressions; no ROS or runtime execution."""
import unittest
from pathlib import Path
from y_place_square_geometry import detect_nearest_topline
import cv2
import numpy as np
from y_place_slot_sides import side_intersections


class SideTests(unittest.TestCase):
    def scene(self):
        frame=np.full((480,640,3),180,np.uint8)
        cv2.line(frame,(230,290),(410,445),(0,220,220),12)
        cv2.line(frame,(410,290),(230,445),(0,220,220),12)
        for x in [210,410]:
            for j,y in enumerate(range(280,480,20)):
                cv2.rectangle(frame,(x,y),(x+20,y+19),
                              (0,220,220) if j%2 else (0,0,0),-1)
        d=dict(x_identity=dict(center_px=[320,360],roi_px=[180,280,300,200]),
               top_edge_px=[[180,280],[480,280]],image_size=[640,480])
        return frame,d

    def test_roi_width_is_not_used_as_top_endpoints(self):
        f,d=self.scene()
        result=side_intersections(f,d)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['top_edge_px'],[[220,280],[420,280]],atol=3)

    def test_adjacent_slot_cannot_replace_right_border_of_selected_x(self):
        source=Path(__file__).with_name('regression_stage.png')
        if not source.exists():
            source=Path(__file__).resolve().parents[1]/'analysis/review_20260907_044602/test_y_20260907_044602_8/stage_reacquisition_raw.png'
        if not source.exists():self.skipTest('saved camera fixture unavailable')
        result=detect_nearest_topline(cv2.imread(str(source)))
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['top_edge_px'],[[287,277],[464,274]],atol=8)

    def test_x_alone_does_not_supply_striped_sides(self):
        f,d=self.scene()
        f[:,200:232]=180
        f[:,405:435]=180
        self.assertIsNone(side_intersections(f,d))

    def test_sides_must_meet_the_observed_top_band(self):
        f,d=self.scene()
        d['warning_tape']=dict(x_min_px=290,x_max_px=350)
        self.assertIsNone(side_intersections(f,d))

    def test_striped_sides_without_interior_x_are_not_a_slot(self):
        f,d=self.scene()
        f[285:470,235:405]=180
        self.assertIsNone(side_intersections(f,d))

    def test_solid_yellow_patch_is_not_an_x(self):
        f,d=self.scene()
        f[290:440,235:405]=(0,220,220)
        self.assertIsNone(side_intersections(f,d))

    def test_dark_yellow_is_not_also_counted_as_black(self):
        f,d=self.scene()
        yellow=(f[:,:,0]==0)&(f[:,:,1]==220)&(f[:,:,2]==220)
        f[yellow]=(0,90,90)
        result=side_intersections(f,d)
        self.assertIsNotNone(result)
        for side in result['side_bands']:
            self.assertLessEqual(side['yellow']+side['black'],1.)
        np.testing.assert_allclose(result['top_edge_px'],[[220,280],[420,280]],atol=3)

    def test_bottom_can_be_outside_image_with_both_sides_visible(self):
        f,d=self.scene()
        f=f[:410]
        d['image_size']=[640,410]
        result=side_intersections(f,d)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['top_edge_px'],[[220,280],[420,280]],atol=3)

    def test_oblique_side_bands_are_not_required_to_be_image_vertical(self):
        f,d=self.scene()
        matrix=np.float32([[.7,.6,-70],[-.1,.65,100],[0,0,1]])
        def warp(points):
            return cv2.perspectiveTransform(np.float32([points]),matrix)[0]
        f=cv2.warpPerspective(f,matrix,(640,480),borderValue=(180,180,180))
        d['x_identity']['center_px']=warp([[320,360]])[0].tolist()
        d['x_identity']['roi_px']=[200,240,250,160]
        d['top_edge_px']=warp([[180,280],[480,280]]).tolist()
        result=side_intersections(f,d)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['top_edge_px'],warp([[220,280],[420,280]]),atol=6)


if __name__=='__main__':unittest.main()
