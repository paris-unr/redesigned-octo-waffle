#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int8MultiArray, Header
from isat_robot_control.msg import MoveArms, MoveArm, MoveHead

class MistyDemo:
    def __init__(self, idx=0):
        # rospy.sleep(20)
        self.ip = None
        while not self.ip:
            self.ip = rospy.get_param("/misty/id_" + str(idx) + "/robot_ip")
            rospy.sleep(1.0)
        
        rospy.init_node("mistyDemo", anonymous=True)
        # PUBLISHERS:
        self.face_img_pub = rospy.Publisher("/misty/id_" + str(idx) + "/face_img", String, latch=True, queue_size=2)
        self.head_pub = rospy.Publisher("/misty/id_" + str(idx) + "/head", MoveHead, latch=True, queue_size=2)
        self.arms_pub = rospy.Publisher("/misty/id_" + str(idx) + "/arms", MoveArms, latch=True, queue_size=2)
        self.speech_pub = rospy.Publisher("/misty/id_" + str(idx) + "/speech", String, latch=True, queue_size=2)
        self.led_pub = rospy.Publisher("/misty/id_" + str(idx) + "/led", Int8MultiArray, latch=True, queue_size=2)

        # greeting = "\
        #     <speak> \
        #         <prosody pitch='high' volume='low'> \
        #             <p>Hello everyone! My name is Misty. I am currently receiving all my control \
        #             commands via the Robot Operating System, or ROS! </p> \
        #             <p> Using this popular middle ware means that I can communicate with all kinds \
        #             of packages. It also means that we'll have easy conversion to file types \
        #             that I can send to the Woz ware servers. \
        #         </prosody> \
        # </speak>\
        # "

        rospy.sleep(10.0)

        msg = String()
        msg.data = "e_Joy.jpg"
        self.face_img_pub.publish(msg)

        left_arm_cmd = MoveArm(85, 100.0)
        right_arm_cmd = MoveArm(-85, 100.0)
        arm_cmd = MoveArms(Header(), "degrees", left_arm_cmd, right_arm_cmd)
        self.arms_pub.publish(arm_cmd)
        rospy.sleep(0.5)

        left_arm_cmd = MoveArm(-85, 100.0)
        right_arm_cmd = MoveArm(85, 100.0)
        arm_cmd = MoveArms(Header(), "degrees", left_arm_cmd, right_arm_cmd)
        self.arms_pub.publish(arm_cmd)
        rospy.sleep(0.5)

        head_cmd = MoveHead()
        head_cmd.header = Header()
        head_cmd.units = "degrees"
        head_cmd.roll = 35.0
        head_cmd.pitch = 0.0
        head_cmd.yaw = 0.0
        head_cmd.velocity = 100.0
        self.head_pub.publish(head_cmd)

        greeting = "<speak> \
                <prosody pitch='high' volume='x-low'> \
                    <p>Hello everyone! My name is Misty.\
                </prosody> \
        </speak>"

        # msg = String(greeting)
        # self.speech_pub.publish(msg)

        left_arm_cmd = MoveArm(75, 20.0)
        right_arm_cmd = MoveArm(75, 20.0)
        arm_cmd = MoveArms(Header(), "degrees", left_arm_cmd, right_arm_cmd)
        self.arms_pub.publish(arm_cmd)

        rospy.sleep(0.75)
        head_cmd.roll = 0.0
        self.head_pub.publish(head_cmd)

        msg = String()
        msg.data = "e_Joy2.jpg"
        self.face_img_pub.publish(msg)
        rospy.sleep(2.0)
        msg.data = "e_DefaultContent.jpg"
        self.face_img_pub.publish(msg)

if __name__ == "__main__":
    MistyDemo()
