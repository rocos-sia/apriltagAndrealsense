
import math
from queue import Empty
from this import d
from time import sleep
import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from apriltag_ros.msg import AprilTagDetectionArray
import rtde_control
import rtde_receive
import time
from scipy.spatial.transform import Rotation as Ro
# rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
# rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")

class apriltag:
    def __init__(self):
        self.mark_flag = 0  # 累加标志位
        self.pose_flag = Pose()  # 累加位姿
        self.div = 30
        self.flag = 0
        self.send_command = [0, 0, 0, 0, 0, 0]
    def quat2T(self,a, b, c, d, e, f, g):
        r = Ro.from_quat([d, e, f, g])
        rotation = r.as_matrix()
        translation = np.array([a, b, c])
        one = np.array([0, 0, 0, 1])
        t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
        T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
        return T
    def rot2T(self,pose):
        r = Ro.from_rotvec([pose[3], pose[4], pose[5]])
        rotation = r.as_matrix()
        translation = np.array([pose[0], pose[1], pose[2]])
        one = np.array([0, 0, 0, 1])
        t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
        T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
        return T
    def T2rot(self,T):
        end_rotation = T[0:3, 0:3]
        end_translation = T[0:3, 3]
        r = Ro.from_matrix(end_rotation)
        rv = r.as_rotvec()
        rv_end = [end_translation[0], end_translation[1], end_translation[2], rv[0], rv[1], rv[2]]
        return rv_end

    def mark_callback(self, data):
        self.pose_flag.position.x = self.pose_flag.position.x+data.detections[0].pose.pose.pose.position.x
        self.pose_flag.position.y = self.pose_flag.position.y + data.detections[0].pose.pose.pose.position.y
        self.pose_flag.position.z = self.pose_flag.position.z + data.detections[0].pose.pose.pose.position.z
        self.pose_flag.orientation.x = self.pose_flag.orientation.x + data.detections[0].pose.pose.pose.orientation.x
        self.pose_flag.orientation.y = self.pose_flag.orientation.y + data.detections[0].pose.pose.pose.orientation.y
        self.pose_flag.orientation.z = self.pose_flag.orientation.z + data.detections[0].pose.pose.pose.orientation.z
        self.pose_flag.orientation.w = self.pose_flag.orientation.w + data.detections[0].pose.pose.pose.orientation.w
        self.mark_flag = self.mark_flag + 1
        
        if self.mark_flag ==self.div:
            data_t =self.quat2T(self.pose_flag.position.x / self.div, self.pose_flag.position.y / self.div,
                              self.pose_flag.position.z / self.div,
                              self.pose_flag.orientation.x / self.div, self.pose_flag.orientation.y / self.div,
                              self.pose_flag.orientation.z / self.div, self.pose_flag.orientation.w / self.div)
       
        # spin() simply keeps python from exiting until this node is stopped
            # get_pose=rtde_r.getActualTCPPose()
            get_pose=np.array([0.1,0.1,0.1,0.1,0.1,0.1])
            eelink2baselink = self.rot2T(get_pose)
            cammer2eelink = np.eye(4)
            mark2cammer = self.rot2T([0, 0, 0.30, math.pi, 0, 0])
            eelink2baselink_2 = np.dot(np.dot(np.dot(np.dot(eelink2baselink, cammer2eelink),
                                                     data_t), np.linalg.inv(mark2cammer)), np.linalg.inv(cammer2eelink))
            send_command = self.T2rot(eelink2baselink_2)
            self.flag += 1
            for i in range(6):
                self.send_command[i] += send_command[i]
            if self.flag == 6:

                print("send_command=", [self.send_command[0] / 6, self.send_command[1] / 6, self.send_command[2] / 6,
                                        self.send_command[3] / 6, self.send_command[4] / 6,
                                        self.send_command[5] / 6])
                # rtde_r.movel([self.send_command[0] / 6, self.send_command[1] / 6, self.send_command[2] / 6,
                #                   self.send_command[3] / 6,
                #                   self.send_command[4] / 6, self.send_command[5] / 6], 0.05, 0.02, 0, [], "", "", True)
                self.flag = 0
                for i in range(6):
                    self.send_command[i] = 0
                

            # 标志位
            self.mark_flag = 0
            self.pose_flag.position.x = 0
            self.pose_flag.position.y = 0
            self.pose_flag.position.z = 0
            self.pose_flag.orientation.x = 0
            self.pose_flag.orientation.y = 0
            self.pose_flag.orientation.z = 0
            self.pose_flag.orientation.w = 0
    def mark(self):
        print("recognize aruco")
        # 订阅话题，矫正5次
        i = 0
        while i < 900:
            data = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray)
            if (len(data.detections)>0):
                
                self.mark_callback(data)
                i = i + 1
            else:
                print("apriltag没有识别到")
        print("apriltag识别完成")

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    apriltag=apriltag()
    try:
        apriltag.mark()
    except:
        print("error")


