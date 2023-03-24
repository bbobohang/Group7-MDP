import json
import os
import socket
import time
from pprint import pprint
import requests
import ast
import threading
import playsound

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
FIXED_DIST_1 = 30
FIXED_DIST_2 = 33


# Getting the images from RPI------------------------------------------------------
# model = torch.hub.load('.', 'custom', path='best_yolov7.pt', source='local')  
# model = torch.hub.load("WongKinYiu/yolov7","custom","best_yolov7.pt",trust_repo=True)

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
def capture(expected):
    reply = {}
    THRESHOLD = 0.7

    print("Capture function")
    
    """Capture the last image from cv2.videocapture()"""
    cap.open("http://192.168.7.7:5000/stream.mjpg")
    ret, image = cap.read()
    
    # img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.resize(image, (640, 640))

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
    
    # #Send image capture to Bluetooth
    # msg_img = "AN|" + "TARGET," + id + ","+ cls
    # s.send(msg_img.encode())
    # time.sleep(0.5)

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
    print(res)
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
def send_us():
    print("Asking for distance")
    msg = "US|dist"
    s.send(msg.encode())
    return
    # n100 182
    # +- 50 = > 3cm
    # +- 69 = > 5cm
    # +- 182 = > 11.5cm
    # +- 217 = > 15cm

    
# Main logic
expected = {}
try:
    #Await for bluetooth start
    print("Waiting for bluetooth to send arena...")
    # start = s.recv(buffer).decode()
    text = input("Enter to start:")
    
    second_count = 0
    first_turn = "left"
    second_turn = "second"
    #Start US to get distance
    while True:
        print("Starting first obstacle")
        send_us()
        first_dist = s.recv(buffer).decode()
        first_dist = round(float(first_dist))
        if first_dist < 80:
            break
        else:
            send_to_stm("FW050")
            stm_movement_reply()
        print("Dist:",str(first_dist))
    first_dist_abs = abs(first_dist - FIXED_DIST_1)
    first_dist_string = str(first_dist_abs)
    first_dist_string = first_dist_string.zfill(3)
    if(first_dist < FIXED_DIST_1):
        message = "BW" + str(first_dist_string)
    else:
        message = "FW" + str(first_dist_string)
    send_to_stm(message)
    stm_movement_reply()

    # # Snap photo
    # captured = capture(expected)
    # if(captured.get('class') == None):
    #     print("No image detected")
    # elif (str(captured.get('class')) == "38"):
    #     first_turn = "right"
    #     #Call stm right command
    #     send_to_stm("OR001")
    #     stm_movement_reply()
    # else:
    #     #Call stm left command
    #     first_turn = "left"
    #     send_to_stm("OL001")
    #     stm_movement_reply()
    # text = input('next')
    first_turn = "right"
    send_to_stm("OR001")
    stm_movement_reply()

    while True:
        print("Starting second obstacle")
        send_us()
        second_dist = s.recv(buffer).decode()
        second_dist = round(float(second_dist))
        if second_dist < 80:
            break
        else:
            second_count += 1
            send_to_stm("FW050")
            stm_movement_reply()
        print("Dist:",str(second_dist))
    second_dist_detected = second_dist
    second_dist_abs = abs(second_dist - FIXED_DIST_2)
    second_dist_string = str(second_dist_abs)
    second_dist_string = second_dist_string.zfill(3)
    if(second_dist < FIXED_DIST_2):
        message = "BW" + str(second_dist_string)
    else:
        message = "FW" + str(second_dist_string)
    if(second_dist_abs > 1):
        send_to_stm(message)
        stm_movement_reply()
    

    # Snap photo
    # captured = capture(expected)
    # if(captured.get('class') == None):
    #     print("No image detected")
    # elif (str(captured.get('class')) == "38"):
    #     second_turn = "right"
    #     #Call stm right command
    #     send_to_stm("OR002")
    #     stm_movement_reply()
    # else:
    #     #Call stm left command
    #     second_turn = "left"
    #     send_to_stm("OL002")
    #     stm_movement_reply()
    second_turn = "left"
    send_to_stm("OL002")
    stm_movement_reply()
    # text = input('next')
    # 57 first right overshot
    #8 second right turn overshot
    #57 first left 
    #14 second left turn
    total_second_dist = second_count * 50 + second_dist_detected
    #Calculate distance to 10cm infront of 1st obstacle
    #right right
    # final_dist = total_second_dist + 57 - 6 + 20 
    if first_turn == "left" and second_turn == "left":
        final_dist = total_second_dist + 57 - 14 + 20
    elif first_turn == "left" and second_turn == "right":
        final_dist = total_second_dist + 57 - 8 + 20
    elif first_turn == "right" and second_turn == "right":
        final_dist = total_second_dist + 57 - 8 + 20
    else:
        final_dist = total_second_dist + 57 - 14 + 20

    final_dist_str = str(final_dist)
    final_dist_str = final_dist_str.zfill(3)
    final_dist_str = "FW" + final_dist_str
    send_to_stm(final_dist_str)
    stm_movement_reply()
    # text = input('next')
    #Final turn in 
    if second_turn == "left":
        send_to_stm("OR003")
        stm_movement_reply()
    else:
        send_to_stm("OL003")
        stm_movement_reply()

    #Final straight line
    send_us()
    obs_dist = s.recv(buffer).decode()
    obs_dist = round(float(obs_dist))
    obs_dist = obs_dist - 20
    print("Obstacle at:", str(obs_dist))
    obs_dist_str = str(obs_dist)
    obs_dist_str = obs_dist_str.zfill(3)
    message = "FW" + obs_dist_str
    send_to_stm(message)
    stm_movement_reply()
    
#     #Send to stm distance to move to first obstacle and wait for reply
#     #If too close, move back
#     new_distance = abs(first_dist_avg - FIXED_DIST)
#     str_new_distance = str(new_distance)
#     str_new_distance = str_new_distance.zfill(3)
#     if(first_dist_avg < FIXED_DIST):
#         message = "BW" + str(str_new_distance)
#     else:
#         message = "FW" + str(str_new_distance)
#     send_to_stm(message)
#     stm_movement_reply()


#     #Snap photo
#     captured = capture(expected)
#     if(captured.get('class') == None):
#         print("No image detected")
#     elif (str(captured.get('class')) == "38"):
#         #Call stm right command
#         send_to_stm("OR001")
#         stm_movement_reply()

#     else:
#         #Call stm left command
#         send_to_stm("OL001")
#         stm_movement_reply()

#     # send_to_stm("OL001")
#     # stm_movement_reply()
# #------------------------------------------------------------------
#     #Start US to get distance
#     print("Starting second obstacle")
#     send_us()
#     second_dist = s.recv(buffer).decode()
#     second_dist = round(float(second_dist))
#     print("Obstacle at:", str(second_dist))
    
#     #Send to stm distance to move to first obstacle and wait for reply
#     #If too close, move back
#     #TODO :fixed the message sent
#     new_distance = abs(second_dist - FIXED_DIST)
#     str_new_distance = str(new_distance)
#     str_new_distance = str_new_distance.zfill(3)
#     if(second_dist < FIXED_DIST):
#         message = "BW" + str(str_new_distance)
#     else:
#         message = "FW" + str(str_new_distance)
#     send_to_stm(message)
#     stm_movement_reply()

#     final_pos = "left"
#     # Snap photo
#     captured = capture(expected)
#     if(captured.get('class') == None):
#         print("No image detected")
#     elif (str(captured.get('class')) == "38"):
#         final_pos = "left"
#         #Call stm right command
#         send_to_stm("OR002")
#         stm_movement_reply()
#     else:
#         #Call stm left command
#         final_pos = "right"
#         send_to_stm("OL002")
#         stm_movement_reply()
#     # send_to_stm("OL002")
#     # stm_movement_reply()
#     #57 first right overshot
#     #8 second right turn overshot
#     #57 first left 
#     #14 second left turn
        
#     #Calculate distance to 10cm infront of 1st obstacle
#     #right right
#     # final_dist = second_dist + 57 - 6 + 20 

#     #left right
#     # final_dist = second_dist + 57 - 6 + 20 
#     #left left
#     final_dist = second_dist + 57 - 8 + 20
#     final_dist_str = str(final_dist)
#     final_dist_str = final_dist_str.zfill(3)
#     final_dist_str = "FW" + final_dist_str
#     send_to_stm(final_dist_str)
#     stm_movement_reply()

#     #Final turn in 
#     if final_pos == "left":
#         send_to_stm("OL003")
#         stm_movement_reply()
#     else:
#         send_to_stm("OR003")
#         stm_movement_reply()
    

#     #Final straight line
#     send_us()
#     obs_dist = s.recv(buffer).decode()
#     obs_dist = round(float(obs_dist))
#     obs_dist = obs_dist - 20
#     print("Obstacle at:", str(obs_dist))
#     obs_dist_str = str(obs_dist)
#     obs_dist_str = obs_dist_str.zfill(3)
#     message = "FW" + obs_dist_str
#     send_to_stm(message)
#     stm_movement_reply()

except Exception as e:
    print("*ERROR*")
    print(e)  # photo()
# finally:
#     # Display collage
#     photo(expected)