import serial
import threading
import time

class STMInterface:
    def __init__(self, RPiMain):
        self.RPiMain = RPiMain
        self.baudrate = 115200
        self.serial = 0
        self.connected = False
        self.threadListening = False

    def connect(self):
        
        try:
            #Serial COM Configuration
            self.serial = serial.Serial("/dev/ttyUSB0", self.baudrate, write_timeout = 0)
            print("Connected to STM 0 successfully.")
            #set flag to true
            self.connected = True
        except:
            try:
                self.serial = serial.Serial("/dev/ttyUSB1", self.baudrate, write_timeout = 0)
                print("Connected to STM 1 successfully.")
                #set flag to true
                self.connected = True
            except Exception as e2:
                print("Failed to connect to STM: %s" %str(e2))
                self.connected = False

    #msg to other parts from STM
    def listen(self):
        
        #set flag to true
        self.threadListening = True
        
        line = []
        
        #loop for listening
        while True:
            try:
                message = self.serial.read(10)
#                 message = self.serial.read()
                #strmsg = str(message)
                print('Read from STM: %s' %str(message))
                message = str(message)
                length = len(message)
                
                if length <= 1:
                    # print("continue")
                    continue
                
#                 trimmedmsg = message[0:length-2]
#                 #trimmedmsg = trimmedmsg.replace('\x00', '')
#                 print(trimmedmsg)
#                 decodedMsg = trimmedmsg.decode("utf-8")
#                 print('Decoded Msg:' + decodedMsg)
#                 print(decodedMsg.strip('\x08'))
#                 self.RPiMain.PC.send(decodedMsg.strip('\x00'))
#                 self.RPiMain.PC.send(decodedMsg.strip('\x08'))
                if "ACK" in message:
                    self.RPiMain.PC.send('ACK')
        
            except Exception as e:
                print("Failed to read from STM: %s" %str(e))
                self.threadListening = True
                self.connected = True
                return
        
    #msg to STM from other parts
    def send(self, message):
        try:
            encoded_string = message.encode()
            byte_array = bytearray(encoded_string)
            self.serial.write(byte_array)
            print("Write to STM: " + message)

        except Exception as e:
            print("Failed to write to STM: %s" %str(e))
            
def test(STM):
    while True:
        time.sleep(2) #send test msg every 2s if STM is connected
        if STM.connected:
            STM.send(b'x') #change datatype to bytes
            break;
               