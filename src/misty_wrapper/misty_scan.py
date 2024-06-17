#!/usr/bin/env python3

"""SOURCE: CPsridharCP on github. https://github.com/CPsridharCP/MistySkills/blob/master/Apps/Teleop/02_pythonTeleop/findMisty.py"""
from gevent import monkey
monkey.patch_all()


import socket
import grequests
import requests
import json
import PySimpleGUI as sg
import pyperclip

# SCANS NETWORK FOR AVAILABLE MISTYS

class MistyScanner:

    def __init__(self):
        self.self_ip = self.find_self_ip() 
        self.base_ip = self.self_ip[:self.self_ip.rfind(".")+1] 

    def find_self_ip(self):
        print("Getting this device's IP:")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        print("Found Device IP:", ip)
        s.close()
        return ip
    
    def scan_for_misty(self):
        print("Starting scan to find Mistys:")
        urls = []
        mistys_found = []
        
        for i in range (256):
            urls.append('http://' + self.base_ip + str(i) + '/api/device')
        results = grequests.map((grequests.get(u, timeout=0.5) for u in urls), exception_handler=self.exception, size=5)
        for result in results:
            if result != None:
                try:
                    data = json.loads(result.text)
                    mistys_found.append([data["result"]["ipAddress"],data["result"]["macAddress"],data["result"]["serialNumber"]])
                except:
                    print("Skipped")
        
        print ("Number of Misty's Found ", len(mistys_found))
        print (mistys_found)
        return mistys_found

    def exception(self, request, exception):
        # print ("Problem: {}: {}".format(request.url, exception))
        pass


def initial_ip_scan_window():

    misty_ip_to_use = None

    direct_ip_input = [
        [sg.Text("Enter Misty's IP on local network :"), sg.Input(key = "-IP-")],
        [sg.Button("START", key = "START"), sg.Text(visible = False, key = "IP-VALIDITY", size = (50,1))]
    ]

    scanner = [
        [sg.Text("Scan for Misty's on the network")],
        [sg.Button("START SCAN", key = "SCAN"),sg.Text("Status:"), sg.Text("Idle.", key = "STATUS", size = (75,1))]
    ]

    scan_results = [
        [sg.Text("Scan Results", visible = False, key = "SCAN-RESULT-TITLE", size = (75,1))],
        *[[sg.Button(str(i), visible = False, key = "SCAN_RESULT_" + str(i), size = (40,1)),] for i in range(10)],
    ] 
    
    layout = [ 
        [sg.Column(direct_ip_input)],
        [sg.Column(scanner)],
        [sg.Column(scan_results)]
    ]

    w, h = sg.Window.get_screen_size()
    window = sg.Window("Misty Teleop", location=(0, 0), default_button_element_size=(15,2), auto_size_buttons=False, keep_on_top=True).Layout(layout)
    
    misty_scanner_object = MistyScanner()
    scanning_in_progress = False
    mistys_found = None

    while True:

        event, values = window.read(timeout=20)
        if event == "Exit" or event == sg.WIN_CLOSED:
            break
        
        # USER INPUT IP
        if event == "START":
            print(values["-IP-"])
            if (values["-IP-"].strip()):
                print("Checking IP validity..")
                try:
                    response = requests.get(url='http://' + values["-IP-"].strip() + '/api/device', timeout =3).json()
                    if response["status"] == "Success":
                        # IP IS A VLAID MISTY IP
                        print("VALID MISTY IP")
                        misty_ip_to_use = values["-IP-"].strip()
                        break
    
                    else:
                        print("NOT A VALID MISTY IP")
                        window["IP-VALIDITY"].update("This is not a valid Misty IP. Try scanning for one.")
                        window["IP-VALIDITY"].update(visible = True)
                        window.Refresh()
                except:
                    print("NOT A VALID MISTY IP")
                    window["IP-VALIDITY"].update("This is not a valid Misty IP. Try scanning for one.")
                    window["IP-VALIDITY"].update(visible = True)
                    window.Refresh()

            else:
                window["IP-VALIDITY"].update("Please enter a valid Misty IP and click START.")
                window["IP-VALIDITY"].update(visible = True)
                window.Refresh()

        # START SCAN FOR MISTY
        if event == "SCAN":
            print("Starting scan for Misty")
            scanning_in_progress = True
            window["STATUS"].update("Scanning in Progress. Please wait.. May take up to 15 seconds")
            window["SCAN"].update(disabled = True)
            window["START"].update(disabled = True)
            window.Refresh()
            mistys_found = misty_scanner_object.scan_for_misty()
            # mistys_found = [['10.0.0.237', '00:d0:ca:00:97:10', '20193802891'],['10.0.0.155', '00:d0:ca:00:97:10', '20193802891'],['10.0.0.102', '00:d0:ca:00:97:10', '20193802891']]
            # mistys_found = []

        # MISTY SCAN RESULTS
        if mistys_found != None and scanning_in_progress:
            print("Scanning Completed")
            print(mistys_found)
            window["STATUS"].update("Scanning complete.")
            window["START"].update(disabled = False)
            window["SCAN"].update(disabled = False)
            scanning_in_progress = False

            if len(mistys_found):
                window["SCAN-RESULT-TITLE"].update("Scan Results - Click on any result to start teleop session")
                window["SCAN-RESULT-TITLE"].update(visible = True)
                for i in range(len(mistys_found)):
                    window["SCAN_RESULT_" + str(i)].update("IP: " + mistys_found[i][0] + "  S.No.: " + mistys_found[i][2])
                    window["SCAN_RESULT_" + str(i)].update(visible = True)
            else:
                window["SCAN-RESULT-TITLE"].update("Scan Results - No Misty found on the same network as this device. Try again")
                window["SCAN-RESULT-TITLE"].update(visible = True)
        
        # SELECT MISTY FROM SCAN RESULT
        if event.startswith("SCAN_RESULT_"):
            misty_ip_to_use = mistys_found[int(event.replace("SCAN_RESULT_", "").strip())][0]
            break
    
    window.close()
    return misty_ip_to_use

if __name__ == "__main__":
    ip = initial_ip_scan_window()
    pyperclip.copy(ip)
    print("IP copied to clipboard")