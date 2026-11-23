"""Offline crop of XYZ .npy arrays, or a synthetic demonstration."""
import argparse
import numpy as np
from .core import crop_mask,parse_boxes,validate_boxes


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True);sub.add_parser('demo')
    crop=sub.add_parser('crop');crop.add_argument('input');crop.add_argument('output')
    crop.add_argument('--boxes',nargs='+',type=float,required=True);crop.add_argument('--keep-inside',action='store_true')
    a=p.parse_args(argv)
    if a.command=='demo':
        points=np.array([[0.,0.,0.],[2.,0.,0.]])
        boxes=parse_boxes([-1,1,-1,1,-1,1]);kept=points[crop_mask(points,boxes)]
        assert kept.tolist()==[[2.,0.,0.]];print(kept.tolist());return 0
    try:
        points=np.load(a.input,allow_pickle=False)
        if points.ndim!=2 or points.shape[1]!=3:raise ValueError('Expected an Nx3 array')
        boxes=validate_boxes(parse_boxes(a.boxes))
        np.save(a.output,points[crop_mask(points,boxes,remove_inside=not a.keep_inside)])
    except (ValueError,OSError) as error:p.exit(2,str(error)+'\n')
    return 0


if __name__=='__main__':raise SystemExit(main())
