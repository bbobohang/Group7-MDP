# run this program on each RPi to send a labelled image stream
import socket
import time
import threading
import pyshine as ps
import picamera

class ImageRecInterface:
    def __init__(self, RPiMain):
        self.RPiMain = RPiMain
        self.StreamProps = ps.StreamProps
        self.connected = False
        self.threadListening = False

        self.HTML="""
        <html>
        <head>
        <title>Pyshine Live Streaming </title>
        </head>
        
        <body>
        <center><h1>  Live Image Recognition </h1></center>
        <center><img src="stream.mjpg" width='1280' height='720' autoplay playsinline></center>
        </body>
        </html>
        """

    def connect (self):
        try:
            self.StreamProps.set_Page(self.StreamProps, self.HTML)
#             self.address = ('192.168.192.10',5000)
            self.address = ('192.168.7.7', 5000)
            self.StreamProps.set_Mode(self.StreamProps, 'picamera')
            
            
            print("Camera started successfully")
            #set flag to true
            self.connected = True
        except Exception as e:
            print("Failed to start camera: %s" %str(e))
    
    def listen(self):
        
        self.threadListening = True
        
        with picamera.PiCamera(resolution='1280x720', framerate=5) as self.camera:
            self.output = ps.StreamOut()
            self.StreamProps.set_Output(self.StreamProps, self.output)
            self.camera.rotation = 0
            self.camera.start_recording(self.output, format='mjpeg')
            try:
                self.server = ps.Streamer(self.address, self.StreamProps)
                print('Server started at','http://' + self.address[0] + ':' + str(self.address[1]))
                self.server.serve_forever()
            finally:
                self.camera.stop_recording()
            
#     def disconnect(self):
#         try:
            