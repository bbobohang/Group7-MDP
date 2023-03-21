import socket
import time
import json 

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


print("sending")
stringAlgo = '{"cat":"obstacles","obstacles":[{"x":2,"y":12,"id":1,"d":0},{"x":10,"y":6,"id":2,"d":2},{"x":11,"y":18,"id":3,"d":4},{"x":19,"y":15,"id":4,"d":6},{"x":15,"y":9,"id":5,"d":0},{"x":18,"y":2,"id":6,"d":6},{"x":5,"y":6,"id":7,"d":6}],"mode":"0"}'
print(connect_algo_server(stringAlgo))
