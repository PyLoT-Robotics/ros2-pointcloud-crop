"""Optional isolated DDS tests with synthetic publishers; no hardware access."""
import os
import signal
import subprocess
import sys
import time
import json
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class Harness:
    def __init__(self,package,domain,parameters):
        self.context=Context();rclpy.init(context=self.context,domain_id=domain)
        self.node=Node(package+'_test',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
        args=[sys.executable,'-c',f'from {package}.node import main; main()','--ros-args']
        for key,value in parameters.items():args+=['-p',f'{key}:={value}']
        self.process=subprocess.Popen(args,env=self.env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)

    def wait(self,predicate,publish=None,timeout=8.):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            assert self.process.poll() is None,self.process.stdout.read().decode()
            if publish:publish()
            self.executor.spin_once(timeout_sec=.025)
        assert predicate()

    def call(self,name):
        from std_srvs.srv import Trigger
        client=self.node.create_client(Trigger,name);self.wait(client.service_is_ready)
        future=client.call_async(Trigger.Request());self.wait(future.done)
        return future.result()

    def close(self):
        try:os.killpg(self.process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=2)
        output=self.process.stdout.read().decode();self.process.stdout.close()
        self.executor.shutdown();self.node.destroy_node();rclpy.shutdown(context=self.context)
        assert self.process.returncode==0,output

def test_real_pointcloud_filter_preserves_intensity_and_uses_tf():
    from sensor_msgs.msg import PointCloud2,PointField
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    import struct
    h=Harness('pointcloud_crop',81,{'input_topic':'/crop_test/input','output_topic':'/crop_test/output','box_frame':'base_link'})
    received=[]
    try:
        h.node.create_subscription(PointCloud2,'/crop_test/output',lambda m:received.append(m),qos_profile_sensor_data)
        pub=h.node.create_publisher(PointCloud2,'/crop_test/input',qos_profile_sensor_data)
        tf=StaticTransformBroadcaster(h.node);t=TransformStamped()
        t.header.frame_id='base_link';t.child_frame_id='crop_laser';t.transform.translation.x=1.;t.transform.rotation.w=1.;tf.sendTransform(t)
        def send():
            msg=PointCloud2();msg.header.frame_id='crop_laser';msg.header.stamp=h.node.get_clock().now().to_msg()
            msg.height=1;msg.width=2;msg.point_step=16;msg.row_step=32
            msg.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z','intensity'])]
            msg.data=struct.pack('<ffffffff',-1.,0.,0.,11.,2.,0.,0.,22.);pub.publish(msg)
        h.wait(lambda:bool(received),send)
        out=received[-1]
        assert out.width==1 and out.header.frame_id=='crop_laser'
        assert struct.unpack('<ffff',bytes(out.data))==(2.,0.,0.,22.)
    finally:h.close()
