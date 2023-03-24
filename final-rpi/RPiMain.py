# from TFLite_detection_webcamV3 import *
from Android import *
from STM import *
from PC import *
from ImageRec import *
import time
import threading
from multiprocessing import Process, Queue
from bluetooth import *

class RPiMain():
    
    def run(self):
        #Run PC Interface
        self.PC = PCInterface(self)
        #Run Android Interface
        self.Android = AndroidInterface(self)
        #Run STM Interface
        self.STM = STMInterface(self)
        #Run ImageRec Interface
        self.ImageRec = ImageRecInterface(self)
        self.Lock = threading.Lock()
        while True:
            
            # Android control loop
            if self.Android.connected == False:
                self.Android.connect()
            elif self.Android.connected == True:
                if self.Android.threadListening == False:
                    try:
                        threading.Thread(target=self.Android.listen).start() # start Android socket listener thread
                    except Exception as e:
                        print("Android threading error: %s" %str(e))
                        self.Android.connected = False
            

             # ImageRec control loop
            if self.ImageRec.connected == False:
                self.ImageRec.connect()
            elif self.ImageRec.connected == True:
                if self.ImageRec.threadListening == False:
                    try:
                        threading.Thread(target=self.ImageRec.listen).start() # start ImageRec socket listener thread
                    except Exception as e:
                        print("ImageRec threading error: %s" %str(e))
                        self.ImageRec.connected = False
                        
            # PC control loop
            if self.PC.connected == False:
               self.PC.connect()
            elif self.PC.connected == True:
               if self.PC.threadListening == False:
                   try:
                        threading.Thread(target=self.PC.listen).start() # start PC socket listener thread
            #                         self.PC.send("PC connected to RPI ")
            #                         if self.Android.connected == True:
            #                             self.PC.send("Bluetooth connected to RPI")
                   except Exception as e:
                       print("PC threading error: %s" %str(e))
                       self.PC.connected = False
                                        
            # STM control loop
            if self.STM.connected == False:
                self.STM.connect()
            elif self.STM.connected == True:
                if self.STM.threadListening == False:
                   try:
                       threading.Thread(target=self.STM.listen).start() # start STM listener thread\
                   except Exception as e:
                       print("STM threading error: %s" %str(e))
                       self.STM.connected = False
             
            
                        
                        
    def disconnectAll(self):
        try:
            self.Android.disconnect()
            self.PC.disconnect()
        except:
            pass


rpi = RPiMain()
rpi.run()
