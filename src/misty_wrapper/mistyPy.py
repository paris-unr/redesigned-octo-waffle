    # /**********************************************************************
    # 	Original Copyright 2020 Misty Robotics
    # 	Licensed under the Apache License, Version 2.0 (the "License");
    # 	you may not use this file except in compliance with the License.
    # 	You may obtain a copy of the License at
    # 		http://www.apache.org/licenses/LICENSE-2.0
    # **********************************************************************/

import base64
import requests
import json
import threading
import time
import websocket

try:
    import thread
except ImportError:
    import _thread as thread
from random import*

class Robot:

    def __init__(self,ip):
        self.ip = ip

        self.images_saved = []
        self.audio_saved  = []
        self.faces_saved = []

        self.backpack_instance = None
        self.time_of_flight_instance = [None]*4
        self.face_recognition_instance = None

        self.available_subscriptions = ["SerialMessage", "TimeOfFlight","FaceRecognition","LocomotionCommand","HaltCommand","SelfState","WorldState"]

        self.populateImages()
        self.populateAudio()
        # self.populateLearnedFaces()

        self.valid_picture_resolutions = [(4160, 3120), (3840, 2160), (3264, 2448), (3200, 2400),
            (2592, 1944), (2048, 1536), (1920, 1080), (1600, 1200), (1440, 1080), (1280, 960), (1024, 768),
            (800, 600), (640, 480), (320, 240)]

        self.valid_video_resolutions = [(3840, 2160), (1920, 1080), (1280, 960), (640, 480), (320, 240)]
        self.valid_stream_resolutions = [(1920, 1280), (1280, 960), (640, 480), (320, 240)]

    def ChangeLED(self,red,green,blue):
        assert red in range(0,256) and blue in range(0,256) and green in range(0,256), " ChangeLED: The colors need to be in 0-255 range"
        requests.post('http://'+self.ip+'/api/led',json={"red": red,"green": green,"blue": blue})

    def DisplayImage(self,image_name,timeout=5):
        if image_name in self.images_saved:
            requests.post('http://' + self.ip + '/api/images/display',json={'FileName': image_name ,'TimeOutSeconds': 5,'Alpha': 1})
        else:
            print(image_name,"not found on the robot, use <robot_name>.printImageList() to see the list of saved images")

    def playAudio(self,file_name):
        if file_name in self.audio_saved:
            requests.post('http://' + self.ip + '/api/audio/play',json={"AssetId": file_name})
        else:
            print(file_name,"not found on the robot, use <robot_name>.printAudioList() to see the list of saved audio files")

    def uploadAudio(self, file_name, apply=False, overwrite=False):
        url = 'http://' + self.ip + '/api/audio'	
        with open(file_name, 'rb') as f:			
            encoded_string = base64.b64encode(f.read()).decode('ascii')				
            data={"FileName": file_name, "Data": encoded_string, "ImmediatelyApply" : apply, "OverwriteExisting": overwrite}			
            requests.post(url, json=data)		

    def battery(self):
        resp = requests.get('http://' + self.ip + '/api/battery')
        for reply in resp.json():
            return (reply['result'])

    def MoveHead(self,roll,pitch,yaw,velocity=10, units="degrees", duration=None):
        if(units == "position"):
            assert -5.0 <= roll <= 5.0 and -5.0 <= pitch <= 5.0 and -5.0 <= yaw <= 5.0, " moveHead: Roll, Pitch and Yaw needs to be in range -5 to +5"
        elif(units == "radians"): # TODO fix these values
            assert -.75 <= roll <= .75 and -.1662 <= pitch <= .6094 and -1.57 <= yaw <= 1.57, " moveHead: invalid positioning"
        else:
            units = "degrees"
            assert -40 <= pitch <= 26 and -40 <= roll <= 40 and -81 <= yaw <= 81, " moveHead: invalid positioning"

        assert 0.0 <= velocity <= 100.0, " moveHead: Velocity needs to be in range 0 to 100"
        json={"Pitch": pitch, "Roll": roll, "Yaw": yaw, "Velocity": velocity, "Units": units}
        # if duration:
        #     json["Duration"] = duration
        requests.post('http://'+self.ip+'/api/head',json=json)

    def moveHeadPosition(self, pitch, roll, yaw, velocity):
        self.MoveHead(pitch, roll, yaw, velocity, "position")
    
    def moveHeadRadians(self, pitch, roll, yaw, velocity):
        self.MoveHead(pitch, roll, yaw, velocity, "radians")
    
    def moveHeadDegrees(self, pitch, roll, yaw, velocity):
        self.MoveHead(pitch, roll, yaw, velocity, "degrees")

    def drive(self,linear_velocity, angular_velocity):
        assert -100 <= linear_velocity <= 100 and -100 <= angular_velocity <= 100, " drive: The velocities needs to be in the range -100 to 100"
        requests.post('http://'+self.ip+'/api/drive',json={"LinearVelocity": linear_velocity,"AngularVelocity": angular_velocity})

    def driveTime(self,linear_velocity, angular_velocity,time_in_milli_second):
        assert -100 <= linear_velocity <= 100 and -100 <= angular_velocity <=100, " drive: The velocities needs to be in the range -100 to 100"
        assert isinstance(time_in_milli_second, int) or isinstance(time_in_milli_second, float), " driveTime: Time should be an integer or float and the unit is milli seconds"
        json = {"LinearVelocity": linear_velocity,"AngularVelocity": angular_velocity, "TimeMS": time_in_milli_second}
        
        requests.post('http://'+self.ip+'/api/drive/time',json = {"LinearVelocity": linear_velocity,"AngularVelocity": angular_velocity, "TimeMS": time_in_milli_second})

    def driveTrack(self,left_track_speed,right_track_speed):
        assert -100 <= left_track_speed <= 100 and right_track_speed in -100 <= right_track_speed <= 100, " driveTrack: The velocities needs to be in the range -100 to 100"
        requests.post('http://'+self.ip+'/api/drive/track',json={"LeftTrackSpeed": left_track_speed,"RightTrackSpeed": right_track_speed})
    
    def stop(self):
        requests.post('http://'+self.ip+'/api/drive/stop')
        
    def sendBackpack(self,message):
        assert isinstance(message, str), " sendBackpack: Message sent to the Backpack should be a string"
        requests.post('http://'+self.ip+'/api/serial',json={"Message": message})

    def populateImages(self):
        self.images_saved = []
        resp = requests.get('http://'+self.ip+'/api/images/list')
        if "result" in resp.json().keys() :   
            for out in resp.json()["result"]:  
                self.images_saved.append(out["name"])

    def populateAudio(self):
        self.audio_saved = []
        resp = requests.get('http://'+self.ip+'/api/audio/list')
        for out in resp.json()["result"]:
            self.audio_saved.append(out["name"])

    def populateLearnedFaces(self):
        self.faces_saved = []
        resp = requests.get('http://'+self.ip+'/api/faces')
        for out in resp.json()["result"]:
            self.faces_saved.append(out)

    def printImageList(self):
        print(self.images_saved)
    
    def getImageList(self):
        return self.images_saved

    def printAudioList(self):
        print(self.audio_saved)
    
    def getAudioList(self):
        return self.audio_saved
    
    def printSubscriptionList(self):
        print(self.available_subscriptions)

    def startFaceRecognition(self):
        requests.post('http://' + self.ip + '/api/faces/recognition/start')
    
    def stopFaceRecognition(self):
        requests.post('http://'+self.ip+'/api/faces/recognition/stop')

    def printLearnedFaces(self):
        print(self.faces_saved)

    def getLearnedFaces(self):
        return self.faces_saved

    def clearLearnedFaces(self):
        requests.delete('http://'+self.ip+'/api/faces')
        self.faces_saved = []

    def MoveArm(self, arm, position, velocity, units = "degrees"):
        arm = arm.lower()
        units = units.lower()

        assert arm == "left" or arm == "right", "Invalid arm requested. Please use 'left' or 'right'"
        assert units == "degrees" or units == "radians" or units == "position", "Invalid unit. Please use 'degrees', 'radians', or 'position'"
        assert 0 < velocity <= 100, "Velocity should be between 0 and 100"

        if(units == "degrees"):
            assert -90 < position <= 90, "Expected value -90< <=90"
        elif(units == "radians"):
            assert -1.5708 < position <= 1.5708, "Expected value -1.5708< <=1.5708"
        else:
            assert 0 <= position <= 10, "Expected value 0 <= >= 10"

        requests.post('http://'+self.ip+'/api/arms', json={"Arm": arm, "Position": position, "Velocity": velocity, "Units": units})

    def MoveArmDegrees(self, arm, position, velocity):
        self.MoveArm(arm, position, velocity, "degrees")

    def MoveArmPosition(self, arm, position, velocity):
        self.MoveArm(arm, position, velocity, "postion")
    
    def MoveArmRadians(self, arm, position, velocity):
        self.MoveArm(arm, position, velocity, "radians")

    def MoveArms(self, rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity, units = "degrees"):
        units = units.lower()

        assert units == "degrees" or units == "radians" or units == "position", "Invalid unit. Please use 'degrees', 'radians', or 'position'"
        assert 0 < rightArmVelocity <= 100 and 0 < leftArmVelocity <= 100, "Velocity should be between 0 and 100"

        if(units == "degrees"):
            assert -90 < rightArmPosition <= 90 and -90 < leftArmPosition <= 90, "Expected value -90< <=90"
        elif(units == "radians"):
            assert -1.5708 < rightArmPosition <= 1.5708 and -1.5708 < leftArmPosition <= 1.5708, "Expected value -1.5708< <=1.5708"
        else:
            assert 0 <= rightArmPosition <= 10 and 0 <= leftArmPosition <= 10, "Expected value 0 <= >= 10"

        requests.post('http://' + self.ip + '/api/arms/set', json={"leftArmPosition": leftArmPosition, "rightArmPosition": rightArmPosition, "leftArmVelocity": leftArmVelocity, "rightArmVelocity": rightArmVelocity, "units": units})

    def MoveArmsDegrees(self, rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity):
        self.MoveArms(rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity, "degrees")
    
    def MoveArmsPosition(self, rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity):
        self.MoveArms(rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity, "position")
    
    def MoveArmsRadians(self, rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity):
        self.MoveArms(rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity, "radians")

    def learnFace(self,name):
        assert isinstance(name, str), " trainFace: name must be a string"
        requests.post('http://'+self.ip+'/api/faces/training/start',json={"FaceId": name})
        print("Please look at Misty's face for 15 seconds..")
        for i in range(15):
            print(15-i)
            time.sleep(1)
        print("Face Captured!!")
        print("Please allow 15 second processing time !")
        for i in range(15):
            print(15-i)
            time.sleep(1)
        print("Face Trained")
        self.populateLearnedFaces()

    ##### WEB SOCKETS #####

    def backpack(self):
        if self.backpack_instance is not None:
            data = self.backpack_instance.data
            try:
                return json.loads(data)["message"]["message"]
            except:
                return json.loads(data)

        else:
            return " Backpack data is not subscribed, use the command robot_name.subscribe(\"SerialMessage\")"

    def time_of_flight(self):
        if self.time_of_flight_instance[0] is not None or self.time_of_flight_instance[1] is not None or self.time_of_flight_instance[2] is not None or self.time_of_flight_instance[3] is not None:

            out = "{"
            for i in range(4):
                try:
                    data_out = json.loads(self.time_of_flight_instance[i].data)
                    #print(data_out)
                    out+="\""+data_out["message"]["sensorPosition"]+"\""+":"
                    out+=str(data_out["message"]["distanceInMeters"])+","
                except:
                    return json.loads(self.time_of_flight_instance[i].data)
            out = out[:-1]
            out+="}"
            return json.loads(out)
        else:
            return " TimeOfFlight not subscribed, use the command robot_name.subscribe(\"TimeOfFlight\")"

    def faceRec(self):
        data = json.loads(self.face_recognition_instance.data)
        try:
            out = "{ \"personName\" : \"" + data["message"]["personName"] + "\", \"distance\" : \"" + str(data["message"]["distance"]) + "\", \"elevation\" :\"" + str(data["message"]["elevation"]) + "\"}"
            return(json.loads(out))
        except:
            return json.loads(self.face_recognition_instance.data)

    def subscribe(self,Type,value=None,debounce =0):
        assert isinstance(Type, str), " subscribe: type name need to be string"

        if Type in self.available_subscriptions:

            if Type == "SerialMessage":
                if self.backpack_instance is  None:
                    self.backpack_instance = Socket(self.ip,Type,_value=value, _debounce = debounce)
                    time.sleep(1)

            elif Type ==  "TimeOfFlight":
                if self.time_of_flight_instance[0] is None:
                    self.time_of_flight_instance[0] = Socket(self.ip,Type,_value="Left", _debounce = debounce)
                    time.sleep(0.05)
                    self.time_of_flight_instance[1] = Socket(self.ip,Type,_value="Center", _debounce = debounce)
                    time.sleep(0.05)
                    self.time_of_flight_instance[2] = Socket(self.ip,Type,_value="Right", _debounce = debounce)
                    time.sleep(0.05)
                    self.time_of_flight_instance[3] = Socket(self.ip,Type,_value="Back", _debounce = debounce)
                    time.sleep(1)

            elif Type == "FaceRecognition":
                if self.face_recognition_instance is None:
                    self.startFaceRecognition()
                    print("FaceRecStarted")
                    self.face_recognition_instance = Socket(self.ip,Type,_value="FaceRecognition", _debounce = debounce)
                
        else:
            print(" subscribe: Type name - ",Type,"is not recognized by the robot, use <robot_name>.printSubscriptionList() to see the list of possible Type names")
    
    def unsubscribe(self,Type):
        assert isinstance(Type, str), " unsubscribe: type name need to be string"

        if Type in self.available_subscriptions:

            if Type == "SerialMessage":

                if self.backpack_instance is not None:
                    self.backpack_instance.unsubscribe()
                    self.backpack_instance = None
                else:
                    print("Unsubscribe:",Type, "is not subscribed")

            elif Type ==  "TimeOfFlight":
                
                if self.time_of_flight_instance[0] is not None:
                    for i in range(4):
                        self.time_of_flight_instance[i].unsubscribe()
                        time.sleep(0.05)
                    self.time_of_flight_instance = [None]*4
                else:
                    print("Unsubscribe:",Type,"is not subscribed")

            if Type == "FaceRecognition":

                if self.face_recognition_instance is not None:
                    self.face_recognition_instance.unsubscribe()
                    self.face_recognition_instance = None
                    self.stopFaceRecognition()
                else:
                    print("Unsubscribe:",Type, "is not subscribed")

        else:
            print(" unsubscribe: Type name - ",Type,"is not recognised by the robot, use <robot_name>.printSubscriptionList() to see the list of possible Type names")

# added

    def Speak(self, ssml_string, flush=False, uid=0):
        json = {
            "Text" : ssml_string,
            "Flush": flush,
            "UtteranceID": uid
        }
        return requests.post("http://" + self.ip + "/api/tts/speak", json=json)
    
    def captureSpeech(self, require_key_phrase=False, overwrite_existing=True, max_length = 7500, silence_timeout=5000):
        json = {
            "RequireKeyPhrase": require_key_phrase,
            "OverwriteExisting": overwrite_existing,
            "MaxSpeechLength" : max_length,
            "SilenceTimeout": silence_timeout
        }
        return requests.post("http://" + self.ip + "/api/audio/speech/capture", json=json)
    
    def startAvStream(self, url, dimensions=(1920, 1080), framerate=30, video_bitrate=5000000, audio_bitrate=128000,
        audio_samplerate=44100, username=None, password=None):
        """
        NOTE: to use Misty as her own media server, use rtspd:<port-number>
        """
        # assert dimensions in self.valid_stream_resolutions, "Invalid stream resolution"
        resp = requests.post("http://"+self.ip+"/api/services/avstreaming/enable", json={})
        print(resp.json())
        json = {
            "URL": url,
            "Width": dimensions[0],
            "Height": dimensions[1],
            "FrameRate": framerate,
            "VideoBitRate": video_bitrate,
            "AudioBitRate": audio_bitrate,
            "AudioSampleRateHz": audio_samplerate,
        }

        if username and password:
            json["UserName"] = username
            json["Password"] = password

        return requests.post("http://" + self.ip + "/api/avstreaming/start", json=json)
    
    def stopAvStream(self):
        return requests.post("http://" + self.ip + "/api/avstreaming/stop", json={})

    def startRecordingAudio(self, filename):
        json = {"FileName": filename}
        return requests.post("http://" + self.ip + "/api/audio/record/start", json=json)

    def stopRecordingAudio(self):
        return requests.post("http://" + self.ip + "/api/audio/record/stop", json={})

    def startRecordingVideo(self, filename, mute=False, duration=30, dimensions=(1920, 1080)):
        assert dimensions in self.valid_video_resolutions, "Invalid video resolution"

        json = {
            "FileName": filename,
            "Mute": mute,
            "Duration": duration,
            "Width": dimensions[0],
            "Height": dimensions[1]
            }
        return requests.post("http://" + self.ip + "/api/videos/recordings/start", json=json)

    def stopRecordingVideo(self):
        return requests.post("http://" + self.ip + "/api/videos/recordings/stop", json={})

    def takePicture(self, filename, return_base64_data=True, dimensions =(800, 600), display_on_screen=True,
        overwrite_existing=True):
        """
        :return: (if return_base_64=True) a Result object containing image data and meta info:
            Base64 (string) - A string containing the Base64-encoded image data.
            ContentType (string) - The type and format of the image returned.
            Height (integer) - The height of the image in pixels.
            Name (string) - The name of the image.
            Width (integer) - The width of the image in pixels.
        """

        assert dimensions in self.valid_picture_resolutions, "Invalid picture resolution"

        json = {
            "Base64": return_base64_data,
            "FileName": filename,
            "Width": dimensions[0],
            "Height": dimensions[1],
            "DisplayOnScreen": display_on_screen,
            "OverwriteExisting": overwrite_existing
        }
        return requests.post("http://" + self.ip + "/api/cameras/rgb", json=json)

# Every web socket is considered an instance     
class Socket:

    def __init__(self, ip,Type, _value = None, _debounce = 0):

        self.ip = ip
        self.Type  = Type
        self.value = _value 
        self.debounce = _debounce
        self.data = "{\"status\":\"Not_Subscribed or just waiting for data\"}"
        self.event_name = None
        self.ws = None
        self.initial_flag = True
    
        dexter = threading.Thread(target=self.initiate)
        dexter.start()

    def initiate(self):
        websocket.enableTrace(True)
        self.ws = websocket.WebSocketApp("ws://"+self.ip+"/pubsub",on_message = self.on_message,on_error = self.on_error,on_close = self.on_close)
        self.ws.on_open = self.on_open
        self.ws.run_forever(ping_timeout=10)

    def on_message(self,ws,message):
        if self.initial_flag:
            self.initial_flag = False
        else:
            self.data = message
        
    def on_error(self,ws, error):
        print(error)

    def on_close(self,ws):
        ws.send(str(self.get_unsubscribe_message(self.Type)))
        self.data = "{\"status\":\"Not_Subscribed or just waiting for data\"}"
        ws.close()
        print("###",self.Type," socket is closed ###")

    def on_open(self,ws):
        def run(*args):
            self.ws.send(str(self.get_subscribe_message(self.Type)))
        thread.start_new_thread(run, ())

    def unsubscribe(self):
        self.on_close(self.ws)

    def get_subscribe_message(self,Type):

        self.event_name = str(randint(0,10000000000))

        if Type == "SerialMessage":

            subscribeMsg = {
                "Operation": "subscribe",
                "Type": "SerialMessage",
                "DebounceMs": self.debounce,
                "EventName": self.event_name,
                "Message": "",
                "ReturnProperty": "SerialMessage"}

        elif Type == "TimeOfFlight":

            subscribeMsg = {
            "$id" : "1",
            "Operation": "subscribe",
            "Type": "TimeOfFlight",
            "DebounceMs": self.debounce,
            "EventName": self.event_name,
            "Message": "",
            "ReturnProperty": "",
            "EventConditions":
            [{
                "Property": "SensorPosition",
                "Inequality": "=",
                "Value": self.value
            }]}
        
        elif Type == "FaceRecognition":

            subscribeMsg = {
                "Operation": "subscribe",
                "Type": self.value,
                "DebounceMs": self.debounce,
                "EventName": self.event_name,
                "Message": "",
                "ReturnProperty": ""}

        return subscribeMsg

    def get_unsubscribe_message(self,Type):

        if Type == "SerialMessage":

            unsubscribeMsg = {
                "Operation": "unsubscribe",
                "EventName": self.event_name,
                "Message": ""}

        elif Type == "TimeOfFlight":
            
            unsubscribeMsg = {
                "Operation": "unsubscribe",
                "EventName": self.event_name,
                "Message": ""}

        elif Type == "FaceRecognition":
            
            unsubscribeMsg = {
                "Operation": "unsubscribe",
                "EventName": self.event_name,
                "Message": ""}
        
        return unsubscribeMsg
