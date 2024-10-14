# --------------- Class Description --------------- #


# ------------------------------------------------- #


# ROS2 Imports
import struct
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs_py.point_cloud2 as pc2

# Python Imports
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree


# PointCloud Merger Class
class PointCloudMerger(Node):

    def __init__(self, node_name:str, lidar_pc_topic:str, rgbd_pc_topic:str):
        super().__init__(node_name)
        
        # Subscribers
        self.lidar_pc_sub = self.create_subscription(
            PointCloud2,
            lidar_pc_topic,
            self.lidar_sub_callback,
            10
        )
        
        self.rgbd_pc_sub = self.create_subscription(
            msg_type=PointCloud2,
            topic=rgbd_pc_topic,
            callback=self.rgbd_sub_callback,
            qos_profile=10
        )

        # Publishers
        self.merged_pc_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic="/merged_point_cloud",
            qos_profile=10
        )   
        self.get_logger().info("here1")
        self.timer_period = 0.5
        self.merged_sub_timer = self.create_timer(self.timer_period, self.merged_timer_callback)
        
        self.lidar_pc = None
        self.rgbd_pc = None


    # Subscriber callbacks
    def lidar_sub_callback(self, msg):
        self.lidar_pc = msg
        self.merge_point_clouds(self.lidar_pc, self.rgbd_pc)
    
    def rgbd_sub_callback(self, msg):
        self.rgbd_pc = msg
    
    # Publisher callbacks
    def merged_timer_callback(self):
        msg = PointCloud2()
        self.merged_pc_pub.publish(msg)

    def read_pc2_points(self, point_cloud, type:str):

        if type == "lidar":
            return pc2.read_points_numpy(point_cloud, skip_nans=True, field_names=("x", "y", "z"))

        elif type == "rgb":
            return pc2.read_points_numpy(point_cloud, skip_nans=True, field_names=("x", "y", "z", "rgb"))
        else: 
            print("Invalid input type")
            return -1

    def create_pointcloud2(self, points):

        header = self.lidar_pc.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32,count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32,count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32,count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32,count=1),
        ]
        pointcloud = pc2.create_cloud(header, fields, points)

        return pointcloud
    
    def merge_point_clouds(self, lidar_pc, rgbd_pc):
        
        # Check that LiDAR pc and RGB-D pc are not empty
        if lidar_pc is None or rgbd_pc is None:
            self.get_logger().info("Input clouds are empty")
            return -1
        
        lidar_points = self.read_pc2_points(lidar_pc, "lidar")
        rgbd_points = self.read_pc2_points(rgbd_pc, "rgb")

        rgb_tree = KDTree(rgbd_points[:,:3])

        merged_points = []

        for lidar_p in lidar_points:
            distance, idx = rgb_tree.query(lidar_p[:3], k=1)

            if (distance < 0.05):
                nearest_rgb_point = rgbd_points[idx]
                # self.get_logger().info("RGB: " + str(nearest_rgb_point[3]))
                # self.get_logger().info("RGB uint32: " + str(np.uint32(nearest_rgb_point[3])))

                rgb_value = nearest_rgb_point[3]
                rgb_int = struct.unpack('I', struct.pack('f', rgb_value))[0]

                # Extract individual R, G, B values from packed int
                r = (rgb_int >> 16) & 0x0000ff
                g = (rgb_int >> 8) & 0x0000ff
                b = rgb_int & 0x0000ff

                self.get_logger().info(f"Unpacked RGB: R={r}, G={g}, B={b}")

                packed_rgb = (r << 16) | (g << 8) | b

                rgb_value = nearest_rgb_point[3]
                merged_points.append((nearest_rgb_point[0], nearest_rgb_point[1], nearest_rgb_point[2], packed_rgb))
            else:
                merged_points.append((*lidar_p,0))
        
        merged_cloud = self.create_pointcloud2(merged_points)

        self.merged_pc_pub.publish(merged_cloud)

        return 0
    

def main(args=None):
    rclpy.init(args=args)

    pc_merger_node = PointCloudMerger(
        lidar_pc_topic="/cloud_map", 
        rgbd_pc_topic="/map_camera/cloud_map", 
        node_name="lol"
        )

    rclpy.spin(pc_merger_node)
    
    pc_merger_node.destroy_node()

    rclpy.shutdown()

if __name__=="__main__":
    main()