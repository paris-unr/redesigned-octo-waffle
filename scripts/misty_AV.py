#!/usr/bin/env python3
import rospy
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

from misty_wrapper.mistyPy import Robot
# from mistyPy import Robot

from misty_wrapper.py3_cv_bridge import cv2_to_imgmsg # workaround for cv_bridge incompatibility with Python 3
from misty_wrapper.simple_av_client import VidStreamer
from misty_wrapper.msg import DetectedFace


class MistyAVNode:
    """
    Enables Misty's audio-video streaming function and publishes camera data to ROS
    """

    def __init__(self, idx=0):
        self.ip = ""
        while not self.ip:
            self.ip = rospy.get_param("/misty/id_" + str(idx) + "/robot_ip")
            rospy.sleep(1.0)
        # self.stream_res = (rospy.get_param("/misty_AV_" + str(idx) + "/stream_resolution/W"), rospy.get_param("/misty_AV_" + str(idx) + "/stream_resolution/H"))
        self.stream_res = (640, 480)
        # Publish BGR8-encoded CV stream to ROS as image msg
        self.cam_pub = rospy.Publisher("/misty/id_" + str(idx) + "/camera/color/image_raw", Image, queue_size=10)
        # self.caminfo_pub = rospy.Publisher("misty/id_" + str(idx) + "/camera_info", CameraInfo, queue_size=10)
        # self.face_pub = rospy.Publisher("/misty/id_" + str(idx) + "/face_recognition", DetectedFace, queue_size=10)

        self.robot = Robot(self.ip)

        rospy.init_node("misty_AV_" + str(idx), anonymous=False)

        self._av_setup()
        # self._face_setup()

        while not rospy.is_shutdown():
            self.video_cb()
            # self.face_cb()
            rospy.sleep(0.01)

        self._cleanup()

    def _cleanup(self):
        # self.vid_stream.stop()
        self.robot.stopAvStream()
        self.robot.unsubscribe("FaceRecognition")
        # self.robot.DisableAvStreamingService()
        rospy.logdebug("AV streaming succesfully cleaned up")

    def _av_setup(self, port_no="1999"):
        url = "rtsp://" + self.ip + ":" + port_no
        misty_stream_url = "rtspd:" + port_no
        # w, h = self.stream_res
        # self.robot.EnableAvStreamingService()
        # self.robot.StartAvStreaming(url=url, width=w, height=h, frameRate=30, videoBitRate=5000000,
        #     audioBitRate=128000, audioSampleRateHz=44100)
        print(self.robot.startAvStream(url=misty_stream_url, dimensions=self.stream_res).json()) # TODO switch to RTSPD
        rospy.sleep(2)
        self.vid_stream     = VidStreamer(url).start()
        # AudioPlayer(url).start()

    def _face_setup(self):
        self.robot.subscribe("FaceRecognition")

    def video_cb(self):
        frame = self.vid_stream.frame
        if frame is None or not frame.any():
            rospy.sleep(0.5)
            return
        frame = imutils.rotate_bound(frame, 90)
        img_msg = cv2_to_imgmsg(frame)
        self.cam_pub.publish(img_msg)

    def face_cb(self):
        data = self.robot.faceRec()
        print(data)
        try:
            msg = DetectedFace({"name": data["personName"], "distance" : data["distance"],
            "elevation" : data["elevation"]})
        except KeyError:
            return
        self.face_pub.publish(msg)
        rospy.logdebug('Found face with name {} at distance {}, elevation {}', msg.name, msg.distance, msg.elevation)

    def learn_face_cb(self, name):
        self.robot.learnFace(name)

if __name__ == "__main__":
    MistyAVNode()
