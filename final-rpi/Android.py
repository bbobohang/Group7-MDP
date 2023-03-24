# from bluetooth import *
import bluetooth as bt
import time
import threading
import socket
import select
import sys
import subprocess

class AndroidInterface:
    def __init__(self, RPiMain):
        self.RPiMain = RPiMain
        self.host = "192.168.7.7"
        self.uuid = "49930a2c-04f6-4fe6-beb7-688360fc5995"
#         self.uuid = "00001101-0000-1000-8000-00805f9b34fb"
        self.connected = False
        self.threadListening = False

    def connect(self):

        #Establish and bind socket
        self.socket = bt.BluetoothSocket(bt.RFCOMM)
        print("Android socket established successfully.")
    
        try:
            self.port = self.socket.getsockname()[1] #4
            print("Waiting for connection on RFCOMM channel %d" % self.port)
            self.socket.bind((self.host, bt.PORT_ANY)) #bind to port
            print("Android socket binded successfully.")
            
            #Turning advertisable
            subprocess.call(['sudo', 'hciconfig', 'hci0', 'piscan'])
            self.socket.listen(128)
            
            bt.advertise_service(self.socket, "MDP-TEAM7", service_id = self.uuid, service_classes = [self.uuid, bt.SERIAL_PORT_CLASS], profiles = [bt.SERIAL_PORT_PROFILE],)
        
        except socket.error as e:
            print("Android socket binding failed: %s" %str(e))
            sys.exit()
            
        print("Waiting for Android connection...")
        #self.socket.listen(128)
        try:
            self.client_socket, self.address = self.socket.accept()
            
        except socket.error as e:
            print('Disconnecting...')
            self.disconnectForced()
        print("Connected to Android successfully.")
        self.connected = True
        return 1

    def disconnect(self):
        try:
            #self.client_socket.close()
            self.socket.close()
            #self.socket = -1
            self.connected = False
            self.threadListening = False
            print("Disconnected from Android successfully.")
        except Exception as e:
            print("Failed to disconnect from Android: %s" %str(e))
            
    def disconnectForced(self):
        try:
            self.socket.close()
            self.connected = False
            self.threadListening = False
            print("Disconnected from Android successfully.")
        except Exception as e:
            print("Failed to disconnect from Android: %s" %str(e))
        sys.exit(0)

    def listen(self):
    
        self.threadListening = True
        
        while True:
            try:
                message = self.client_socket.recv(1024)
#                 print("Read from Android: %s" %str(message))

                if not message:
                    print("Android disconnected remotely.")
                    break

                # 3. Parse message  by delimiter
                decodedMsg = message.decode("utf-8")
                
                if len(decodedMsg) <= 1:
                    continue
                print("Read from Android: " + decodedMsg)
                self.RPiMain.PC.send(decodedMsg)
#                 parsedMsg = decodedMsg.split(',')
#                 id = parsedMsg[0]
#                 
#                 if id == 'PC':
#                     androidmsg = parsedMsg[1:]
#                     finalanmsg = ','.join(androidmsg)
#                     self.RPiMain.PC.send(finalanmsg)
                    
                continue


            except socket.error as e:
                print("Failed to read from Android: %s" %str(e))
                break

            except IOError as ie:
                print("Failed to read from Android: %s" %str(ie))
                break

            except Exception as e2:
                print("Failed to read from Android: %s" %str(e2))
                break

            except ConnectionResetError:
                print("ConnectionResetError")
                break

            except:
                print("Unknown error")
                break
                
        #end of listening loop - set flags to false
        self.threadListening = False
        self.connected = False

    def send(self, message):
        try:
            self.client_socket.send(message)
            print("Write to Android: %s" %str(message))
        except Exception as e:
            print("Failed to write to Android: %s" %str(e))
            self.disconnect()

def RepeatingMessageTest(Android):
    while True:
        time.sleep(2) #send test msg every 2s if Android is connected
        if Android.connected:
            Android.send(b'Hi from rpi')
        else:
            print("no connected")
            break
