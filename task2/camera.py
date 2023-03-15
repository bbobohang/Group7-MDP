#To test image rec accuracy on local camera
import cv2
import torch
import time
import os 
import socket

image_dict = {'11': '1', '12': '2', '13': '3', '14': '4', '15': '5', '16': '6', '17': '7', '18': '8', '19': '9', "20": "A", "21":"B", "22": "C", "23": "D", "24": "E", "25":"F", "26": "G", "27": "H", "28": "S", "29": "T", "30": "U", "31": "V", "32": "W","33": "X", "34": "Y", "35": "Z", "36": "UP", "37" : "DOWN", "38": "RIGHT", "39": "LEFT", "40": "STOP"} 

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture()
model = torch.hub.load('.', 'custom', path='best_v3.pt', source='local')  # local repo
print("===== Model loaded =====")

host = "192.168.7.7"
# host = "192.168.192.10"
port = 12345
buffer = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
print("Socket Connected")

# def capture(turn):
#     # time.sleep(5)
#     print("Capture function")
#     ret, image = cap.read()
    
#     image = cv2.resize(image, (640, 640))
#     image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     # recognition
#     results = model(image)
#     results.render()
#     class_dict = results.names
#     boxes = results.xywh[0]
#     boxes = boxes.tolist()
#     print(boxes)
#     if len(boxes) == 0:
#         print("Nothing captured")
#         return
    
#     res = []
#     for box in boxes:
#         if box[4] > 0.7:
#             res.append(box)

#     if len(res) > 0:
#         biggest_box = res[0]
#         for box in res:
#             if box[2] * box[3] > biggest_box[2] * biggest_box[3]:
#                 biggest_box = box
#     else:
#         print("Low accuracy image")
#         return
    
#     x, y, w, h, conf, cls_num = biggest_box
#     x, y, w, h, conf, cls = int(x), int(y), int(w), int(h), round(conf, 2), class_dict.get(int(cls_num))

    
#     if(os.path.exists(f"./camera_testing/{str(cls)}") == False):
#         os.makedirs(f"./camera_testing/{str(cls)}")

#     cv2.imwrite(f"./camera_testing/{str(cls)}/{turn}.png",
#                         results.ims[0])
    
#     return
    
    
def capture():
    reply = {}
    THRESHOLD = 0.5
    cap.open("http://192.168.7.7:5000/stream.mjpg")

    print("Capture function")
    
    """Capture the last image from cv2.videocapture()"""
    ret, image = cap.read()
    
    # img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # img_gray = cv2.resize(image, (640, 640))

    # recognition
    results = model(image)
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
        if box[4] > THRESHOLD:
            res.append(box)
        else:
            print("Box 4:" + str(box[4]))

    #Remove the bulleyes
    for i in range(len(res)):
        detected_class = class_dict.get(int(res[i][5]))
        if(detected_class == "41"):
            res.pop(i)

    # """If there are multiple objects detected, return the biggest bounding box"""
    if len(res) > 0:
        # biggest_box, mid = res[0], abs(int(res[0][0] - 308))
        # for box in res:
        #     midpoint = abs(int(box[0] - 308))
        #     if midpoint < mid:
        #         biggest_box, mid = box, midpoint
        #     elif box[2] * box[3] > biggest_box[2] * biggest_box[3]:
        #         biggest_box, mid = box, midpoint
        biggest_box = res[0]
        
        for box in res:
            if box[2] * box[3] > biggest_box[2] * biggest_box[3]:
                biggest_box = box
    else:
        #Image detected but low accuracy
        print("Low accuracy image")
        return None
    
    # Print out the x1, y1, w, h, confidence, and class of predicted object
    x, y, w, h, conf, cls_num = biggest_box
    cls = str(int(cls_num))

    x, y, w, h, conf, cls = int(x), int(y), int(w), int(h), round(conf, 2), image_dict.get(class_dict.get(int(cls)))
    print("Found: {}, {}, {}, {}, {}, {}".format(x, y, w, h, conf, cls))

    if(os.path.exists(f"./detected_images/{str(cls)}") == False):
            os.makedirs(f"./detected_images/{str(cls)}")

    cv2.imwrite(f"./detected_images/{str(cls)}/conf{str(conf)}_width{w}_height{h}.png",
                results.ims[0])

count = 0
while True:
    captured = capture()
    time.sleep(3)