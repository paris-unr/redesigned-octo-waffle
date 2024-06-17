import cv2
import numpy as np
from threading import Thread

"""
Based heavily on Github user @CPsridharCP's implementation of Misty teleoperation: https://github.com/CPsridharCP/MistySkills/tree/master/Apps/Teleop/02_pythonTeleop

"""

class VidStreamer:
    def __init__(self, path):
        self.stream = cv2.VideoCapture(path)
        self.stopped = False
        self.frame = np.array([])

        # Another option would be to handle the video through python av as well;
        # for now i'm sticking with cv2 to allow for any OpenCV applications we might
        # want to use down the line.

    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # while not self.stopped:
        #     grabbed, frame = self.stream.read()
        #     # if not grabbed:
        #     #     return

        #     self.frame = frame

        # self.stream.release()
        while not self.stopped:
            ret_val, self.frame = self.stream.read()
        self.stream.release()
    def stop(self):
        self.stopped = True


# import av
# # import numpy as np
# # from threading import Thread
# import pyaudio
# from collections import deque
# import time


# class AudioPlayer:

#     def __init__(self, path):
#         self.stream_path = path


#     def update(self):
#         queue = deque()

#         def play_thread():
#             p = pyaudio.PyAudio()
#             stream = p.open(format=pyaudio.paFloat32,
#                             channels=1,
#                             rate=11025,
#                             output=True)
#             while True:
#                 if len(queue) == 0:
#                     time.sleep(0.25)
#                     continue
#                 frame = queue.popleft()
#                 stream.write(frame.to_ndarray().astype(np.float32).tostring())

#         t = Thread(target=play_thread)
#         t.start()  

#         container = av.open(self.stream_path)
#         input_stream = container.streams.get(audio=0)[0]
#         for frame in container.decode(input_stream):
#             frame.pts = None
#             queue.append(frame)

#     def start(self):

#         t = Thread(target=self.update, args=())
#         t.daemon = True
#         t.start()

#     # def stop(self):
#     #     self.stopped = True


if __name__ == "__main__":
    # AudioPlayer("rtsp://192.168.50.156:193").start()
    VidStreamer("rtsp://192.168.50.50:1936").start()