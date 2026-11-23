import struct
import numpy as np
import pytest
from pointcloud_crop.core import filter_records,parse_boxes,validate_boxes
BOX=parse_boxes([-1,1,-1,1,-1,1])
FIELDS=[dict(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]


@pytest.mark.parametrize('endian',['<','>'])
def test_padding_endian_and_extra_fields_survive(endian):
    first=struct.pack(endian+'fffI',0.,0.,0.,111)
    second=struct.pack(endian+'fffI',2.,0.,0.,222)
    data=first+b'pad!'+second+b'pad!'
    result,count=filter_records(data,1,2,16,20,FIELDS,BOX,is_bigendian=endian=='>')
    assert count==1 and result==second


def test_float64_and_reordered_layout():
    fields=[dict(name=n,offset=i*8,datatype=8,count=1) for i,n in enumerate('zyx')]
    data=struct.pack('<ddd',0.,0.,2.)
    result,count=filter_records(data,1,1,24,24,fields,BOX)
    assert count==1 and result==data


@pytest.mark.parametrize('change',['truncated','offset','count','datatype','width'])
def test_malformed_cloud_rejected(change):
    data=bytes(12);fields=[dict(f) for f in FIELDS];width=1
    if change=='truncated':data=b''
    elif change=='offset':fields[0]['offset']=100
    elif change=='count':fields[0]['count']=2
    elif change=='datatype':fields[0]['datatype']=2
    else:width=2
    with pytest.raises(ValueError):filter_records(data,width,1,12,12,fields,BOX)


@pytest.mark.parametrize('bounds',[[1,-1,0,1,0,1],[0,float('nan'),0,1,0,1]])
def test_invalid_boxes(bounds):
    with pytest.raises(ValueError):validate_boxes(parse_boxes(bounds))
