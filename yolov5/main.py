import json
import os
import socket
import time
from pprint import pprint
import requests
import ast
import threading


import cv2
# to plot detected images as collage
import numpy as np
import torch
from PIL import Image

#  algo
# import algorithm

#  config
image_dict = {'11': '1', '12': '2', '13': '3', '14': '4', '15': '5', '16': '6', '17': '7', '18': '8', '19': '9', "20": "A", "21":"B", "22": "C", "23": "D", "24": "E", "25":"F", "26": "G", "27": "H", "28": "S", "29": "T", "30": "U", "31": "V", "32": "W","33": "X", "34": "Y", "35": "Z", "36": "UP", "37" : "DOWN", "38": "RIGHT", "39": "LEFT", "40": "STOP"} 
direction_dict = {0: 'U', 2: 'R', 4: 'D', 6: 'L'}

# Getting the images from RPI------------------------------------------------------
#Original dataset
# model = torch.hub.load('.', 'custom', path='best.pt', source='local')  # local repo
# print("===== Model loaded =====")

# #Low accuracy for 7,Z,H,C,T,4
# #Trained with good dataset, grayscaled 3 times
# model = torch.hub.load('.', 'custom', path='best_good_gray.pt', source='local')  
# print("===== Model loaded =====")

# #Trained with poor dataset, grayscaled
# model = torch.hub.load('.', 'custom', path='best_good_gray.pt', source='local')  
# print("===== Model loaded =====")

# #Trained with good dataset, colored 2 times
# #Low accuracy for 7,4,T,F
# model = torch.hub.load('.', 'custom', path='best_color_2.pt', source='local')  
# print("===== Model loaded =====")

# #Trained with good dataset with missing class, gray 2 times
model = torch.hub.load('.', 'custom', path='gray_3_v3.pt', source='local')  
print("===== Model loaded =====")
#---------------------------------------------------------------------------------

# Socket
host = "192.168.7.7"
# host = "192.168.192.10"
port = 12345
buffer = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
print("Socket Connected")
print("Waiting for bluetooth to send arena...")


# path = []
# pillar_image = []

# # Clean up the pillars to feed algo
# pillars = []
# indoor = 1
# # indoor = int(json.loads(msg)['value']['mode'])
# obs = json.loads(msg)['value']['obstacles']
# for pillar in obs:
#     print(pillar)
#     pillars.append((pillar['x'], pillar['y'], direction_dict[pillar['d']]))
# print(f'Pillars: {pillars}')
# car_path, visiting_order = algorithm.routes_generator((1, 1, 'U'), pillars)
# print(f"Visiting order: {visiting_order}")


# def get_pillars_id(pillars, visiting_order):
#     print(pillars)
#     print(visiting_order)
#     # Write your code here
#     pillar_list = []
#     for order in visiting_order:
#         for pillar in pillars:
#             if order[0] == pillar['x'] and order[1] == pillar['y']:
#                 pillar_list.append(pillar['id'])
#                 break
#     return pillar_list


# ordering = get_pillars_id(obs, visiting_order)

# # visiting_order = [ (7, 17, 'D'), (12, 15, 'R'), (9, 8, 'L'), (5, 0, 'U'), (15, 2, 'U')]

# print(f"Output commands: {car_path}")
# print(f"Ordering: {ordering}")

# Global dir to keep the best image for stitching
# Start capturing of image from server
cap = cv2.VideoCapture()
# cap.open("http://192.168.192.10:5000/stream.mjpg")

# take image, recognize and store it.
def capture(expected, id):
    reply = {}
    THRESHOLD = 0.7

    print("Capture function")
    
    """Capture the last image from cv2.videocapture()"""
    cap.open("http://192.168.7.7:5000/stream.mjpg")
    ret, image = cap.read()
    
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.resize(img_gray, (640, 640))

    # recognition
    results = model(img_gray)
    results.render()

    """Class Dict"""
    class_dict = results.names

    # """Filter out predictions with confidence less than 0.7"""
    #Convert each prediction to a list
    res = []
    boxes = results.xywh[0]
    boxes = boxes.tolist()
    
    #Nothing detected, empty tensor
    if len(boxes) == 0:
        print("Nothing captured")
        return None
    
    """xywh: x,y coordiaante of the center of the bounding box. w,h width height of the bounding box"""
    for box in boxes:
        # Image above midpoint, and small => False
        # if box[1] > 231 and box[3] < 50:
        #     continue
        # # Filter by confidence level
        # elif box[4] > THRESHOLD:
        #     res.append(box)
        print(box)
        if box[4] > THRESHOLD:
            res.append(box)
    
    new_res = []
    #Remove the bulleyes
    for i in range(len(res)):
        detected_class = class_dict.get(int(res[i][5]))
        if(detected_class != "41"):
            new_res.append(res[i])

    # """If there are multiple objects detected, return the biggest bounding box"""
    if len(new_res) > 0:
        # biggest_box, mid = res[0], abs(int(res[0][0] - 308))
        # for box in res:
        #     midpoint = abs(int(box[0] - 308))
        #     if midpoint < mid:
        #         biggest_box, mid = box, midpoint
        #     elif box[2] * box[3] > biggest_box[2] * biggest_box[3]:
        #         biggest_box, mid = box, midpoint
        biggest_box = new_res[0]
        
        for box in new_res:
            if box[2] * box[3] > biggest_box[2] * biggest_box[3]:
                biggest_box = box
    else:
        #Image detected but low accuracy
        print("Low accuracy image")
        return None
    
    # Print out the x1, y1, w, h, confidence, and class of predicted object
    x, y, w, h, conf, cls_num = biggest_box
    cls = str(int(cls_num))

    x, y, w, h, conf, cls = int(x), int(y), int(w), int(h), round(conf, 2), class_dict.get(int(cls))
    print("Found: {}, {}, {}, {}, {}, {}".format(x, y, w, h, conf, cls))
    
    #Send image capture to Bluetooth
    msg_img = "AN|" + "TARGET," + id + ","+ cls
    s.send(msg_img.encode())
    time.sleep(0.5)

#     """how much is the median_detected off from the median_landscape; l is negative, r is positive"""
    median_landscape = 640 / 2
    median_detected = x
    median_diff = median_detected - median_landscape
    median_diff = int(median_diff)
    print("median_diff: ", median_diff)

    reply = {'x': x, 'y': y, 'w': w, 'h': h, 'conf': conf, 'class': cls, 'median': median_diff,
                'class_num': cls, "median_diff": median_diff}

    # Initialize
    if cls not in expected:
        # Creating an empty folder to store the images of that obstacle
        if(os.path.exists(f"./detected_images/{str(cls)}") == False):
            os.makedirs(f"./detected_images/{str(cls)}")
        
        expected[cls] = [x, y, w, h, conf, cls, median_diff,
                            f"./detected_images/{str(cls)}/conf{str(conf)}_width{w}_height{h}_diff{median_diff}.png"]
        print("*" * 50)
        print(f"Making new directory for class {cls}")
        pprint(expected)
        print("")
        print("")

    # Save the best images metadata
    # Best image is large in bounding box, and has good confidence score (+-0.05)
    #If new image of same id detected and the width(w) and height(h) and conf is more than the original by 0.02 
    #then replace the original
    elif (w >= expected[cls][2] and h >= expected[cls][3]) and (conf >= expected[cls][4] - 0.02):
        expected[cls] = [x, y, w, h, conf, cls, median_diff,
                            f"./detected_images/{str(cls)}/conf{str(conf)}_width{w}_height{h}_diff{median_diff}.png"]
        print("*" * 50)
        print(f"Getting a better image for {cls}")
        pprint(expected)
        print("")
        print("")

    print("*" * 50)
    print("Saving image")
    # Save all predicted images
    cv2.imwrite(f"./detected_images/{str(cls)}/conf{str(conf)}_width{w}_height{h}_diff{median_diff}.png",
                results.ims[0])
    return reply
    


# After the Run ends
def img_reshape(img):
    img = Image.open(img)
    img = img.resize((300, 300))
    img = np.asarray(img)
    return img


def photo(expected):
    """Stitched the images together. The directory of each image is found in the argument expected on dictionary value[7]."""
    list_of_images = []
    for key in expected.keys():
        list_of_images.append(expected[key][7])
    print("Stitching these images: ", list_of_images)
    widths, heights = zip(*(Image.open(i).size for i in list_of_images))
    total_width = sum(widths)
    max_height = max(heights)
    new_im = Image.new('RGB', (total_width, max_height))
    x_offset = 0
    for im in list_of_images:
        im = Image.open(im)
        new_im.paste(im, (x_offset, 0))
        x_offset += im.size[0]
    new_im.save('stitched.png')


def compress(input):
    # Count the occurrences of each element
    output = []
    count = 0
    for i in range(len(input) - 1):
        if input[i] == 'L' or input[i] == 'R':
            output.append((input[i], count + 1))
            count = 0
        elif input[i] == input[i + 1]:
            count += 1
        else:
            output.append((input[i], count + 1))
            count = 0
    output.append((input[-1], count + 1))

    # Convert to command
    direction = None
    if indoor:
        direction = {'F': 'i', 'B': 'k', 'L': 'j', 'R': 'o', }
    if not indoor:
        direction = {'F': 'w', 'B': 's', 'L': 'a', 'R': 'r', }
    final = []
    for ele in output:
        # if direction[ele[0]] == 'a' or direction[ele[0]] == 'r' or direction[ele[0]] == 'j' or direction[ele[0]] == 'o':
        #     cur = f'{direction[ele[0]]}090'
        #     final.append(cur)
        #     final.append("{}013".format(direction['B']))
        if direction[ele[0]] == 'a' or direction[ele[0]] == 'j':
            cur = f'{direction[ele[0]]}090'
            final.append(cur)
            final.append("{}018".format(direction['B']))
        elif direction[ele[0]] == 'r' or direction[ele[0]] == 'o':
            cur = f'{direction[ele[0]]}090'
            final.append(cur)
            final.append("{}013".format(direction['B']))
        else:
            cur = (direction[ele[0]] + str(ele[1] * 10).zfill(3))
            final.append(cur)
    return final

def send_to_stm(command):
    res = "STM|" + command
    s.send(res.encode())
    # msg = s.recv(buffer).decode()
    return

def stm_movement_reply():
    count = 0
    while True:
        count += 1
        msg = s.recv(buffer).decode()
        print(f"MSG : {msg} | COUNT : {count}")
        if "ACK" in msg :
            break
        # print("message type: ", type(msg))
        # print("//////")
        # if msg == " " or msg == "IRT 12158":
        #     continue
        # if msg.isalnum:
        #     x = msg.split("'\'")
        #     print("msg is: ", x[0])
        #     res = int(float(x[0]))
        #     print("INFRA-RED: ", res)
        #     return res
        if msg.isalnum():
            print("MSG: >>>>>>>>>", msg)
        # if msg == "f": # or msg " f" or msg "f ":
        #     break
        if "f" in msg:
            break


def correction(diff):
    # guess = {'50': 3, '69': 5, '182': 11.5, '217': 16}
    # guess = {'182': 11.5}
    pos = 'L' if diff < 0 else 'R'
    # res_key, res_val = min(guess.items(), key=lambda x: (abs(diff) - int(x[0])))

    if abs(diff) > 170:
        print("Doing Auto Correct")
        if pos == 'L':
            return 'm100'
        if pos == 'R':
            return 'n100'
    else:
        return None

    # n100 182
    # +- 50 = > 3cm
    # +- 69 = > 5cm
    # +- 182 = > 11.5cm
    # +- 217 = > 15cm

def move_forward(expected,id):
    print("moving forward")
    send_to_stm('FW01')
    stm_movement_reply()
    captured = capture(expected,id)
    send_to_stm('BW01')
    stm_movement_reply()
    return captured

def move_backward(expected, id):
    print("moving backward")
    send_to_stm('BW01')
    stm_movement_reply()
    captured = capture(expected,id)
    send_to_stm('FW01')
    stm_movement_reply()
    return captured

def connect_algo_server(obstaclesString):
    socketAlgo = socket.socket()
    hostAlgo = "192.168.7.9"
    portAlgo = 6000
    socketAlgo.connect((hostAlgo, portAlgo))

    x = obstaclesString.encode()
    socketAlgo.sendall(x + "\n".encode())   
    message = socketAlgo.recv(1024)
    commands = json.loads(message.decode())
    commands = commands.get("commands")
    
    return commands

    
# Main logic
expected = {}
try:
    # infra = 0
    # direction = None
    # if indoor:
    #     direction = {'F': 'i', 'B': 'k', 'L': 'j', 'R': 'o', }
    # if not indoor:
    #     direction = {'F': 'w', 'B': 's', 'L': 'a', 'R': 'r', }
    # count_obstacle = 0
    # while car_path:
    # if(os.path.exists(f"./detected_images_checklist/") == False):
    #     os.makedirs(f"./detected_images_checklist/")
    obstaclesString = s.recv(buffer).decode()

    # string = """{"cat":"obstacles","obstacles":[{"x":1,"y":11,"id":1,"d":4},{"x":7,"y":7,"id":2,"d":4},{"x":15,"y":1,"id":3,"d":6},{"x":18,"y":9,"id":4,"d":6},{"x":14,"y":14,"id":5,"d":4},{"x":9,"y":17,"id":6,"d":6}],"mode":"0"}"""
    commands = connect_algo_server(obstaclesString)
    print(commands)
    # obstaclesJson = json.loads(obstaclesString)
    # headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
    # req = requests.post('http://localhost:8080/api', json=obstaclesJson)
    # commands = req.json().get('commands')
    # print(commands)
    print("Waiting to start task")
    obstaclesString = s.recv(buffer).decode()

    # commands = ["FR90","FW12","FL90","SNAP6","FW01","FR90","FW01","FL90","FW05","FL90","FR90","FW03","SNAP4","BW03","BL90","FW02","BW01","SNAP5","BW10","BR90","FW03","FR90","BW03","SNAP1","FW06","FR90","BL90","FW02","BW01","SNAP3","FW02","FL90","FW10","FL90","FW04","FR90","BW03","SNAP2","FIN"]
    for command in commands:
        print(command)
        # time.sleep(0.5)
        if "SNAP" in command:
            # time.sleep(2)
            # send_to_stm("RST-")
            # stm_movement_reply()
            id = command[len(command) - 1]
            captured = capture(expected,id)

            # Move forward 1 grid to snap 
            if captured == None:
                captured = move_forward(expected,id)
                if captured == None:
                    captured = move_backward(expected,id)
                    if captured == None:
                        print("Gone obstacle")
            else:
                print("Found this: " + captured.get("class"))
        elif command == "FIN":
            print("ending")
            break
        else:
            send_to_stm(command)
            stm_movement_reply()
    print("Stitching images")
    photo(expected)
  
    # while True:
    #     break
    #     cur_command = car_path.pop(0)
    #     cur_command = compress(cur_command)
    #     print(f"cur: {cur_command}")

    #     # cur_command = ['j090']
    #     # cur_command = ['w010', 's010']
    #     for cmd in cur_command:
    #         # Send command for robot to move
    #         msg_stm = f"STM|{cmd}"
    #         print(f'Executing: {msg_stm}')
    #         s.send(msg_stm.encode())
    #         print('Done msg_stm')
    #         # infra = stm_movement_reply()
    #         stm_movement_reply()
    #         print('Done stm_movement_reply')
        # cmd = "testing android"
        # msg_android = f"AN|ROBOT,{cmd}"
        # print(f'Sending Android: {msg_android}')
        # print("android encoded: ", msg_android.encode())
        # s.send(msg_android.encode())
            # print("waiting reply")
        # time.sleep(0.5)
            # print("done")
   
        # x = input("next: ")
        # Capture image
        # captured = capture(expected, 1)
        # if captured == {}:
            # print("Re-capturing from another 10cm away")
            # print(f"sending: STM|{direction['B']}010")
            # s.send(f"STM|{direction['B']}010".encode())
            # # infra = stm_movement_reply()
            # stm_movement_reply()
            # captured = capture(expected, count_obstacle)
            # s.send(f"STM|{direction['F']}010".encode())
            # infra = stm_movement_reply()
            # stm_movement_reply()
            # if infra != 30:
            #     diff_distance = 30 - infra
            #     if diff_distance > 0:
            #         s.send(f"STM|{dir['F']}{str(diff_distance).zfill(3)}".encode())
            #         infra = stm_movement_reply()
            #     else:
            #         s.send(f"STM|{dir['B']}{str(abs(diff_distance)).zfill(3)}".encode())
            #         infra = stm_movement_reply()

        # if captured == {}:
        #     print("GAVE UP THIS OBSTACLE!")

        # # Send android the cls
        # if captured != {}:
        #     print(f"Image is captured. Identified {int(captured['class_num'])}")
        #     msg_android = f"AN|TARGET,{ordering[count_obstacle]},{int(captured['class_num'])}"
        #     print("sending to android: ", msg_android)
        #     s.send(msg_android.encode())

        #     # captured {'x', 'y', 'w', 'h', 'conf', 'class', 'median', 'class_num', 'median_diff'}
        #     # Autocorrect: move back at an angle 1
        #     diff = captured['median_diff']
        #     # command =command correction(diff)
        #     # if command:
        #     #     s.send(f"STM|{command}".encode())
        #     #     # infra = stm_movement_reply()
        #     #     stm_movement_reply()

        # # Maintenance
        # count_obstacle += 1
        
        
        
###########################################################
#Calling algo server /api for task 1
#Receive obstacles from android and returns list of command
    # obstaclesString = s.recv(buffer).decode()
    # obstaclesJson = json.loads(obstaclesString)
    # headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
    # req = requests.post('http://localhost:8080/api', json=obstaclesJson, headers=headers)
    # if(req.status_code != 200):
    #     print(req.text)
    # else:
    #     commands = req.json().get('commands')
    # print(commands)
    # while True: 
    #     for command in commands:
    #         print(command)
    #         if "SNAP" in command:
    #             captured = capture(expected, 1)
    #             print("Captured Class: " + captured.get("class"))
    #     break
###########################################################

###########################################################
#Calling algo server /checklist for cehcklist A5
#Receive obstacle position from bluetooth and move towards obstacles
#Take photo and if not valid image, turn right
    # obstaclesString = s.recv(buffer).decode()
    # obstaclesJson = json.loads(obstaclesString)
    # headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
    # req = requests.post('http://localhost:8080/checklist', json=obstaclesJson, headers=headers)
    # commands = req.json().get('commands')
    # print(commands)
    # while True: 
    #     for command in commands:
    #         print(command)
    #         send_to_stm(command)
    #         stm_movement_reply()

    #     while True:
    #         captured = capture(expected, 1)
    #         if captured.get("class") == "BULLEYE":
    #             #move 90 degrees
    #             continue
    #         else:
    #             break
    #     break
###########################################################

############################################################
#For checklist A1.3
#For receiving from bt->rpi->pc->rpi->stm
        # try:
        #     msg = s.recv(buffer).decode()
        #     if "cat" in msg:
        #         res = json.loads(msg).get("value")
        #         print('message from bt to rpi to pc:', res)
        #         res = "STM|" + res
        #         s.send(res.encode())
        #     else:
        #         print(msg)
            
        # except Exception as e:
        #     print(e)
##############################################################      

############################################################
#For checklist A2
#Able to detect image from 20 - 50cm and labelled on PC
    # cap = cv2.VideoCapture()
    # cap.open("http://192.168.7.7:5000/stream.mjpg")

    # # Capture image
    # captured = capture(expected, 1)
    # if captured == {}:
    #     break
##############################################################   
        
except Exception as e:
    print("*ERROR*")
    print(e)  # photo()
# finally:
#     # Display collage
#     photo(expected)
