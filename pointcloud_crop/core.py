"""Crop geometry extracted from PyLoT; no ROS dependencies."""
from collections.abc import Mapping
import math
import numpy as np

def quaternion_to_matrix(quaternion) -> np.ndarray:
    """クォータニオン (x, y, z, w) を 3x3 回転行列へ変換する."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = x * x + y * y + z * z + w * w
    if norm == 0.0:
        return np.eye(3)
    scale = 2.0 / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - scale * (yy + zz), scale * (xy - wz), scale * (xz + wy)],
        [scale * (xy + wz), 1.0 - scale * (xx + zz), scale * (yz - wx)],
        [scale * (xz - wy), scale * (yz + wx), 1.0 - scale * (xx + yy)],
    ])

def parse_boxes(values) -> list[dict]:
    """
    フラットな数値列を箱のリストへ変換する.

    Parameters
    ----------
    values : sequence of float
        [min_x, max_x, min_y, max_y, min_z, max_z] を 6 個ずつ並べたもの.

    Returns
    -------
    list of dict
        箱 (min_x/max_x/min_y/max_y/min_z/max_z のキーを持つ dict) のリスト.
        入力が空なら空リスト.

    Raises
    ------
    ValueError
        長さが 6 の倍数でない場合.

    """
    values = list(values)
    if len(values) % 6 != 0:
        raise ValueError(
            'boxes の長さは 6 の倍数でなければなりません '
            f'(実際の長さ: {len(values)}). '
            '[min_x, max_x, min_y, max_y, min_z, max_z] を 6 個ずつ並べてください.'
        )
    boxes = []
    for index in range(0, len(values), 6):
        min_x, max_x, min_y, max_y, min_z, max_z = (
            float(value) for value in values[index:index + 6]
        )
        boxes.append({
            'min_x': min_x, 'max_x': max_x,
            'min_y': min_y, 'max_y': max_y,
            'min_z': min_z, 'max_z': max_z,
        })
    return boxes

def _as_box_list(boxes) -> list:
    """箱の mapping または mapping の列を箱の list にそろえる."""
    if isinstance(boxes, Mapping):
        return [boxes]
    return list(boxes)

def crop_mask(
    xyz, boxes, rotation=None, translation=None, remove_inside=True,
) -> np.ndarray:
    """
    点群座標を箱フレームへ変換し、残す点の真偽マスクを返す.

    Parameters
    ----------
    xyz : array_like, shape (N, 3)
        入力点群の x/y/z (点群フレーム).
    boxes : mapping or sequence of mapping
        min_x/max_x/min_y/max_y/min_z/max_z のキーを持つ箱、またはその列.
    rotation : array_like, shape (3, 3), optional
        点群フレーム -> 箱フレームの回転行列。None なら恒等変換.
    translation : array_like, shape (3,), optional
        点群フレーム -> 箱フレームの並進。None なら 0.
    remove_inside : bool, optional
        True ならどれかの箱の内側を除去、False ならどれかの箱の内側だけ残す.

    Returns
    -------
    numpy.ndarray, shape (N,)
        True の点を残す。NaN は remove_inside=True では箱の外として残る.

    """
    points = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    rotation_matrix = (
        np.eye(3) if rotation is None else np.asarray(rotation, dtype=np.float64)
    )
    translation_vector = (
        np.zeros(3) if translation is None
        else np.asarray(translation, dtype=np.float64)
    )
    transformed = points @ rotation_matrix.T + translation_vector

    x = transformed[:, 0]
    y = transformed[:, 1]
    z = transformed[:, 2]
    inside = np.zeros(points.shape[0], dtype=bool)
    for box in _as_box_list(boxes):
        inside |= (
            (x >= float(box['min_x'])) & (x <= float(box['max_x']))
            & (y >= float(box['min_y'])) & (y <= float(box['max_y']))
            & (z >= float(box['min_z'])) & (z <= float(box['max_z']))
        )
    # NaN 点は inside 判定が False になる (NaN 比較は False) ので自然に残る。
    return ~inside if remove_inside else inside


def validate_boxes(boxes):
    boxes=_as_box_list(boxes)
    if not boxes:raise ValueError('At least one crop box is required')
    for box in boxes:
        for axis in 'xyz':
            low=float(box['min_'+axis]);high=float(box['max_'+axis])
            if not math.isfinite(low) or not math.isfinite(high) or low>high:
                raise ValueError('Box bounds must be finite and ordered')
    return boxes


def filter_records(data,width,height,point_step,row_step,fields,boxes,
                   is_bigendian=False,rotation=None,translation=None,remove_inside=True):
    """Return packed kept records, retaining every byte of every kept point.

    fields is an iterable of {name,offset,datatype,count}. Only XYZ must be
    FLOAT32 (7) or FLOAT64 (8). Row padding, endian and changing layouts are
    handled per message. Invalid layout is an error; points are never guessed.
    """
    validate_boxes(boxes)
    if min(width,height)<0 or point_step<=0 or row_step<width*point_step:
        raise ValueError('Invalid PointCloud2 dimensions/strides')
    if len(data)<height*row_step:raise ValueError('Truncated point records')
    if width*height==0:return b'',0
    raw=np.ndarray((height,width,point_step),np.uint8,buffer=bytes(data),
                   strides=(row_step,point_step,1)).reshape(-1,point_step)
    components=[]
    for axis in 'xyz':
        matched=[f for f in fields if f['name']==axis]
        if len(matched)!=1:raise ValueError('XYZ fields must occur exactly once')
        f=matched[0]
        if f['datatype'] not in (7,8) or f.get('count',1)!=1:raise ValueError('XYZ must be scalar FLOAT32/FLOAT64')
        size=4 if f['datatype']==7 else 8;offset=f['offset']
        if offset<0 or offset+size>point_step:raise ValueError('Field exceeds point_step')
        dtype=np.dtype(('>' if is_bigendian else '<')+('f4' if size==4 else 'f8'))
        components.append(raw[:,offset:offset+size].copy().view(dtype).reshape(-1))
    if rotation is not None:
        r=np.asarray(rotation)
        if r.shape!=(3,3) or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or np.linalg.det(r)<0:
            raise ValueError('Invalid rigid rotation')
    if translation is not None:
        v=np.asarray(translation)
        if v.shape!=(3,) or not np.isfinite(v).all():raise ValueError('Invalid translation')
    keep=crop_mask(np.column_stack(components),boxes,rotation,translation,remove_inside)
    return raw[keep].tobytes(),int(np.count_nonzero(keep))
