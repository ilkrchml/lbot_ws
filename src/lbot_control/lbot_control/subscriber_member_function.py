# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.action import NavigateToPose
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


from rclpy.action import ActionClient
import collections
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis, ObjectHypothesisWithPose
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import torch
from torchvision.models import detection as detection_model
from torchvision.utils import draw_bounding_boxes


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        #subscriber
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detected_objects',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.target_dict = {}

        #rgbd depth info use constatn for now
        self.depth = 1.0

        #camera calibrartion matrix
        self.k = np.array([[1696.802685832259, 0.0, 960.5],
                           [0.0, 1696.802685832259,  540.5],
                           [0., 0., 1.]])

        self.k_inv = np.linalg.inv(self.k)
        
        #gemetry of cost map grid
        self.grid_resolution = 0.05000000074505806
        self.grid_width =  323
        self.grid_height = 234

        self.subscription_odom = self.create_subscription(
            Odometry,#String,
            '/odom',#'/topic',
            #'/local_costmap/costmap_updates',
            self.listener_callback_odom,
            10)
        self.odom = Odometry()
        
        #service client
        #self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        #self.get_logger().info('service , waiting ...')
        #while not self.cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('service not available, waiting again...')
        #self.req = AddTwoInts.Request()
        #self.get_logger().info('service , established ...')
        
        #actrion client
        #self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
    def listener_callback_odom(self, msg):
        pose = msg.pose
        self.odom.pose = msg.pose
        #self.get_logger().info('Odom info received')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def listener_callback(self, msg):
        
        #header = msg.header
        detectionArray = msg
        detections = detectionArray.detections
        x = -1
        y = -1
        if detections:
            #bbox = detectionArray.detections[0].bbox
            #x = bbox.center.position.x
            #y = bbox.center.position.y
            for detection in detectionArray.detections:
                hypothesis = detection.results[0].hypothesis
                name = hypothesis.class_id
                score = hypothesis.score
                if(score > .9):
                    bbox = detection.bbox
                    # centroid
                    x = bbox.center.position.x
                    y = bbox.center.position.y
                    x_size = bbox.size_x
                    y_size = bbox.size_y

                    # one of the bounding vertices
                    b_x = x + x_size/2
                    b_y = y + y_size/2

                    #get the estimated pose of the robot
                    pose = self.odom.pose

                    # get rotation and translation from camera frame
                    # to wrold frame
                    rot, t = self.get_rt(pose) 
                    x_y_1 = np.array([x,y,1.])  
                    bx_by_1 = np.array([b_x, b_y, 1.])

                    #camera frame
                    P_c = self.depth * np.matmul(self.k_inv,x_y_1)  
                    Pb_c = self.depth * np.matmul(self.k_inv,bx_by_1) 
                    

                    #world frame
                    P_w = np.matmul(rot, P_c )  
                    P_w = P_w + t

                    Pb_w = np.matmul(rot, Pb_c )  
                    Pb_w = Pb_w + t

                    #get world location
                    lx = P_w[0]
                    ly = P_w[1]

                    lbx = Pb_w[0]
                    lby = Pb_w[1]

                    #get map coordinates 
                    mx = np.floor(self.grid_height/2. + lx/self.grid_resolution + 0.5)
                    my = np.floor(self.grid_width/2. - ly/self.grid_resolution + 0.5)

                    mbx = np.floor(self.grid_height/2. + lbx/self.grid_resolution + 0.5)
                    mby = np.floor(self.grid_width/2. - lby/self.grid_resolution + 0.5)

                    #save map coordinates of the object detected
                    if not name in self.target_dict.keys():
                        self.target_dict[name] = (mx,my, mbx, mby)
                        self.get_logger().info('Coordinates are: "%s"'% [name, mx, my])
                        np.save('my_file.npy', self.target_dict) 

        #self.get_rt(self.odom.pose)  
        #self.get_logger().info('Coordinates are: "%s"'% [lx, ly])
        #response = self.send_request(int(1), int(2))
        #self.get_logger().info(
        #'Result of add_two_ints: for %d + %d = %d' %
        #(int(1), int(2), response.sum))
    def get_rt(self, pose):
        
        p = pose.pose.position
        quart = pose.pose.orientation
        rx = quart.x
        ry = quart.y
        rz = quart.z
        rw = quart.w
        px = p.x
        py = p.y
        pz = p.z
        t = np.array([px,py,pz])
        r = R.from_quat(np.array([rx, ry, rz, rw]))
        rot = r.as_matrix()
        #r_t = np.eye(4)
        #r_t[:3,:3] = rot
        #r_t[:3,3] = t
        self.get_logger().info('Odom Coordinates are: "%s"'% [rx, ry, rz, rw])
        return rot, t
    def get_coordinates(self, query):
        if query in self.target_dict.keys:
            coord = self.target_dict[query]
        else:
            coord = (-10000, -10000)
        
        return coord
    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose  = pose

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    #navigator = BasicNavigator()

    #test service client
    #response = minimal_subscriber.send_request(int(1), int(2))
    #minimal_subscriber.get_logger().info(
    #    'Result of add_two_ints: for %d + %d = %d' %
    #    (int(1), int(2), response.sum))
    
    #test action client
    #pose = PoseStamped()
    #pose.header.frame_id = 'map'
    #pose.header.stamp = navigator.get_clock().now().to_msg()
    #pose.pose.orientation.w = 1.0
    #pose.pose.position.x = 0.0
    #pose.pose.position.y = 0.0
    #pose.pose.orientation.z = 0.0
    #pose.pose.orientation.w = 1.0

    #future = minimal_subscriber.send_goal(pose)

    #rclpy.spin_until_future_complete(minimal_subscriber, future)

    #minimal_subscriber.get_logger().info(
    #   'Pose action goal is completed')

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
