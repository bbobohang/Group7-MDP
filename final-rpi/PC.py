import socket
import select
import sys
import threading
import time
import RPi.GPIO as GPIO
# from stitch import *
import ultrasonic as us

class PCInterface:
    
    def __init__(self, RPiMain):
        self.TRIG = 23
        self.ECHO = 24
        self.RPiMain = RPiMain
        self.host = "192.168.7.7"
#         self.host = "192.168.192.10"
        self.port = 12345
        self.connected = False
        self.threadListening = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

    def connect(self):

        # 1. Solution for thread-related issues: always attempt to disconnect first before connecting
        self.disconnect()

        # 2. Establish and bind socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print("Socket established successfully.")

        try:
            self.socket.bind((self.host, self.port))
            print("Socket binded successfully.")
        except socket.error as e:
            print("Socket binding failed: %s" %str(e))
            sys.exit()

        # 3. Wait and accept PC connection
        print("Waiting for PC connection...")
        self.socket.listen(128)
        self.client_socket, self.address = self.socket.accept()
        print("PC connected successfully.")

        # 4. Set flag to true
        self.connected = True

    def disconnect(self):
        try:
            self.socket.close()
            self.connected = False
            self.threadListening = False
            print("Disconnected from PC successfully.")
        except Exception as e:
            print("Failed to disconnect from PC: %s" %str(e))

    def listen(self):
        
        # 1. Set flag to true
        self.threadListening = True

        # 2. Loop for listening
        while True:
            try:
                message = self.client_socket.recv(1024)
                #print("Read from PC: %s" %str(message))

                if not message:
                    print("PC disconnected remotely.")
                    break

                # 3. Parse message by delimiter
                decodedMsg = message.decode("utf-8")
                if len(decodedMsg) <= 1:
                    continue
                print("Read from PC: " + decodedMsg)
                parsedMsg = decodedMsg.split('|')
                id = parsedMsg[0]
                
                if id == 'AN':
                    self.RPiMain.Android.send(parsedMsg[1])
                  
                if id == 'STM':
                    #padding to 8 chars to send to STM
#                     msg = parsedMsg[1].ljust(8, '0')
                    msg = parsedMsg[1] + '\0'
                    self.RPiMain.STM.send(msg)
#                     self.RPiMain.STM.send(parsedMsg[1])
                if id =='US':
                    
                    GPIO.output(self.TRIG, True)
                    time.sleep(0.00001)
                    GPIO.output(self.TRIG, False)

                    # Wait for the echo response
                #     start_time = time.time()
                    while GPIO.input(self.ECHO) == 0:
                        start_time = time.time()

                    while GPIO.input(self.ECHO) == 1:
                        end_time = time.time()
                    
                    # Calculate distance
                    duration = end_time - start_time
                    distance = duration * 17150
                    distance = round(distance, 2)
                    print("Distance:" + str(distance))
                    self.RPiMain.PC.send(str(distance))
                    
           

            except socket.error as e:
                print("Failed to read from PC: %s" %str(e))
                break

            except IOError as ie:
                print("Failed to read from PC: %s" %str(ie))
                break

            except Exception as e2:
                print("Failed to read from PC: %s" %str(e2))
                break

            except ConnectionResetError:
                print("ConnectionResetError")
                break

            except:
                print("Unknown error")
                break
        
        # 4. End of listening loop - set flags to false
        self.threadListening = False
        self.connected = False


    def send(self, message):
        try:
            encoded_string = message.encode()
            byte_array = bytearray(encoded_string)
            self.client_socket.send(byte_array)
            print("Send to PC: " + message)
        except ConnectionResetError:
            print("Failed to send to PC: ConnectionResetError")
            self.disconnect()
        except socket.error:
            print("Failed to send to PC: socket.error")
            self.disconnect()
        except IOError as e:
            print("Failed to send to PC: %s" %str(e))
            self.disconnect()
    def measure_distance(self):
        # Set the GPIO pins
        
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        # Wait for the echo response
    #     start_time = time.time()
        while GPIO.input(ECHO) == 0:
            start_time = time.time()

        while GPIO.input(ECHO) == 1:
            end_time = time.time()
        
        # Calculate distance
        duration = end_time - start_time
        print(duration)
        distance = duration * 17150
        distance = round(distance, 2)
        return distance

