"""PointCloud2 adapter. Missing TF or malformed layouts are dropped with warnings."""
import numpy as np
from .core import parse_boxes,validate_boxes,filter_records,quaternion_to_matrix


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import Buffer,TransformListener,TransformException

    class CropNode(Node):
        def __init__(self):
            super().__init__('pointcloud_crop')
            values=self.declare_parameter('boxes',[-.5,.5,-.5,.5,-1.,1.]).value
            self.boxes=validate_boxes(parse_boxes(values))
            self.box_frame=self.declare_parameter('box_frame','').value
            self.remove=self.declare_parameter('remove_inside',True).value
            input_topic=self.declare_parameter('input_topic','/points').value
            output_topic=self.declare_parameter('output_topic','/points/cropped').value
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)
            self.pub=self.create_publisher(PointCloud2,output_topic,qos_profile_sensor_data)
            self.create_subscription(PointCloud2,input_topic,self.cloud,qos_profile_sensor_data)

        def cloud(self,msg):
            try:
                rotation=translation=None
                if self.box_frame and self.box_frame!=msg.header.frame_id:
                    if msg.header.stamp.sec==0 and msg.header.stamp.nanosec==0:raise ValueError('Acquisition stamp is required for TF cropping')
                    # Use the acquisition time, not the latest robot pose.
                    t=self.tf.lookup_transform(self.box_frame,msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
                    q=t.rotation
                    if not np.isfinite([q.x,q.y,q.z,q.w]).all() or sum(v*v for v in [q.x,q.y,q.z,q.w])<1e-12:
                        raise ValueError('Invalid TF quaternion')
                    rotation=quaternion_to_matrix((q.x,q.y,q.z,q.w))
                    translation=np.array([t.translation.x,t.translation.y,t.translation.z])
                data,count=filter_records(msg.data,msg.width,msg.height,msg.point_step,msg.row_step,
                    [dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in msg.fields],
                    self.boxes,msg.is_bigendian,rotation,translation,self.remove)
                out=PointCloud2();out.header=msg.header;out.height=1;out.width=count
                out.fields=msg.fields;out.is_bigendian=msg.is_bigendian;out.is_dense=msg.is_dense
                out.point_step=msg.point_step;out.row_step=count*msg.point_step;out.data=data
                self.pub.publish(out)
            except (ValueError,TypeError,KeyError,TransformException) as error:
                self.get_logger().warning(str(error),throttle_duration_sec=5.)

    rclpy.init(args=args);node=None
    try:node=CropNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
