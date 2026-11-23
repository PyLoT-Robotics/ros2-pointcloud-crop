import math
import numpy as np
import pytest
from pointcloud_crop import core as pointcloud_filter

BOX = {
    'min_x': -0.65, 'max_x': 0.5,
    'min_y': -0.3, 'max_y': 0.5,
    'min_z': -2.0, 'max_z': 2.0,
}

BOXES = [
    {
        'min_x': -0.25, 'max_x': 0.55,
        'min_y': -0.11, 'max_y': 0.51,
        'min_z': -2.0, 'max_z': 2.0,
    },
    {
        'min_x': -0.65, 'max_x': 0.1,
        'min_y': -0.3, 'max_y': 0.3,
        'min_z': -2.0, 'max_z': 2.0,
    },
]

def _yaw_rotation(yaw):
    return pointcloud_filter.quaternion_to_matrix(
        (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    )

def test_identity_transform_removes_inside_and_keeps_outside():
    xyz = np.array([
        [0.0, 0.0, 0.0],
        [0.4, 0.4, 1.0],
        [0.6, 0.0, 0.0],
    ])

    mask = pointcloud_filter.crop_mask(xyz, BOX, remove_inside=True)

    assert mask.tolist() == [False, False, True]

def test_yaw_minus_ninety_and_translation_remove_point_behind_robot():
    rotation = _yaw_rotation(-math.pi / 2.0)
    translation = np.array([0.2, 0.0, 0.15])
    # livox_frame の (0, -0.7, 0) は base_footprint の後方 (-0.5, 0, 0.15).
    # livox_frame の (0, 0.5, 0) は base_footprint の前方 (0.7, 0, 0.15) で箱の外.
    xyz = np.array([
        [0.0, -0.7, 0.0],
        [0.0, 0.5, 0.0],
    ])
    base_xyz = xyz @ rotation.T + translation
    assert base_xyz[0, 0] == pytest.approx(-0.5)
    assert base_xyz[1, 0] == pytest.approx(0.7)

    mask = pointcloud_filter.crop_mask(
        xyz, BOX, rotation, translation, remove_inside=True,
    )

    assert mask.tolist() == [False, True]

def test_nan_points_are_kept_when_removing_inside():
    xyz = np.array([
        [np.nan, 0.0, 0.0],
        [0.0, np.nan, 0.0],
        [0.0, 0.0, 0.0],
    ])

    mask = pointcloud_filter.crop_mask(xyz, BOX, remove_inside=True)

    assert mask.tolist() == [True, True, False]

def test_two_boxes_remove_union_and_keep_points_outside_both():
    xyz = np.array([
        [0.3, 0.2, 0.0],    # 箱1 の内側.
        [-0.5, 0.0, 0.0],   # 箱2 の内側.
        [0.3, -0.2, 0.0],   # 外接長方形の前方右 (どちらの箱にも入らない).
        [-0.5, 0.4, 0.0],   # 外接長方形の後方左 (どちらの箱にも入らない).
    ])

    mask = pointcloud_filter.crop_mask(xyz, BOXES, remove_inside=True)

    assert mask.tolist() == [False, False, True, True]

def test_two_boxes_keep_only_union_when_removing_outside():
    xyz = np.array([
        [0.3, 0.2, 0.0],
        [-0.5, 0.0, 0.0],
        [0.3, -0.2, 0.0],
    ])

    mask = pointcloud_filter.crop_mask(xyz, BOXES, remove_inside=False)

    assert mask.tolist() == [True, True, False]

def test_parse_boxes_rejects_length_not_multiple_of_six():
    for length in (1, 5, 7, 13):
        with pytest.raises(ValueError):
            pointcloud_filter.parse_boxes([0.0] * length)

def test_parse_boxes_splits_flat_values_into_boxes():
    boxes = pointcloud_filter.parse_boxes([
        -0.25, 0.55, -0.11, 0.51, -2.0, 2.0,
        -0.65, 0.1, -0.3, 0.3, -2.0, 2.0,
    ])

    assert boxes == BOXES

def test_quaternion_to_matrix_rotates_yaw_minus_ninety():
    rotation = _yaw_rotation(-math.pi / 2.0)

    np.testing.assert_allclose(
        rotation,
        np.array([
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]),
        atol=1e-12,
    )
