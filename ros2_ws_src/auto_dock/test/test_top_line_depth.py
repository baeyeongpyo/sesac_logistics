"""Synthetic sensor contracts, not a claim of real camera accuracy."""
import math
import numpy as np
import pytest
from auto_dock.top_line_depth import (
    measure_sampled_top_line_depth,
    measure_top_line_depth,
    sample_top_line_depth_image,
)

K=np.array([[500.,0.,320.],[0.,500.,240.],[0.,0.,1.]])
LINE=[220.,240.,420.,240.]

def measure(depth, **kwargs):
    return measure_top_line_depth(LINE,depth,K,np.zeros(5),pitch_deg=0.,**kwargs)

def test_measures_depth_not_pixel_scale():
    near=measure(np.full((480,640),.54))
    far=measure(np.full((480,640),.84))
    assert near['top_center_cm']==pytest.approx([0.,40.])
    assert far['top_center_cm']==pytest.approx([0.,70.])
    assert near['heading_left_deg']==pytest.approx(0.)
    assert near['measurement_source']=='registered_top_line_depth'

def test_tilted_line_direction_comes_from_depth():
    u=np.arange(640)
    z=.64/(1.-math.tan(math.radians(12.))*(u-320)/500)
    p=measure(np.broadcast_to(z,(480,640)))
    assert p['heading_left_deg']==pytest.approx(12.,abs=.05)

def test_camera_pitch_uses_vertical_component_and_fork_offset():
    result=measure_top_line_depth([220,340,420,340],np.full((480,640),.64),K,np.zeros(5),
                                  pitch_deg=-20.,forward_offset_cm=14.,right_offset_cm=1.)
    assert result['forward_cm']==pytest.approx(64*(math.cos(math.radians(20))-.2*math.sin(math.radians(20)))-14)
    assert result['right_cm']==pytest.approx(1.)

def test_sparse_depth_and_short_fragment_rejected():
    depth=np.zeros((480,640));depth[:,290:350]=.64
    with pytest.raises(ValueError,match='insufficient'):
        measure(depth)

def test_outlier_patch_does_not_rotate_line():
    depth=np.full((480,640),.64);depth[:,310:335]=1.2
    p=measure(depth)
    assert p['heading_left_deg']==pytest.approx(0.)
    assert p['forward_cm']==pytest.approx(50.)
    assert p['depth_inlier_samples']<33

def test_missing_one_end_rejected_even_with_interior_support():
    depth=np.full((480,640),.64);depth[:,:270]=0.
    with pytest.raises(ValueError,match='consensus'):
        measure(depth)


@pytest.mark.parametrize('encoding,kind,value', [
    ('16UC1', 'u2', 640),
    ('32FC1', 'f4', .64),
])
@pytest.mark.parametrize('is_bigendian', [False, True])
def test_sparse_image_sampling_preserves_endian_and_padded_stride(
        encoding, kind, value, is_bigendian):
    width, height, padding = 640, 480, 12
    dtype = ('>' if is_bigendian else '<')+kind
    itemsize = np.dtype(dtype).itemsize
    step = width*itemsize+padding
    raw = bytearray(step*height)
    image = np.ndarray((height, width), dtype=np.dtype(dtype), buffer=raw,
                       strides=(step, itemsize))
    image[:] = value
    sampled = sample_top_line_depth_image(
        LINE, raw, width, height, step, encoding, is_bigendian=is_bigendian)
    assert sampled['depth_patches_m'].shape == (33, 9)
    assert sampled['depth_patches_m'].nbytes == 33*9*4
    result = measure_sampled_top_line_depth(
        sampled, K, np.zeros(5), pitch_deg=0.)
    assert result['top_center_cm'] == pytest.approx([0., 50.])
