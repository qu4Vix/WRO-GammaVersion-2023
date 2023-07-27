# read UPD message from the ESP32
# this code should be run on a laptop/rasp pi that is connected to the wifi network created by the ESP32
# note that since you are connecting your laptop to the ESP32's wifi network, you will not be able to access the internet
# you will only be able to communicate with the ESP32
# for a version of this communication code that allows your laptop to still connect to the internet, see https://gist.github.com/santolucito/4016405f54850f7a216e9e453fe81803

import socket
# use ifconfig -a to find this IP. If your pi is the first and only device connected to the ESP32, 
# this should be the correct IP by default on the raspberry pi
LOCAL_UDP_IP = "192.168.144.235"
SHARED_UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
sock.bind((LOCAL_UDP_IP, SHARED_UDP_PORT))
def decodeBytes(data):
    index = 0
    decoded = []
    if len(data) > 1:
        cab = data[0]
        if cab == 4:
            for i in range(1,719, 2):
                d = data[i]<<8 | (data[i+1])
                decoded.append(d)
            return decoded
        else:
            return data
    else:
        return -1

while True:
    data, addr = sock.recvfrom(1000000)
    print("received")
    decoded = decodeBytes(data)
    print(decoded)
"""if __name__ == "__main__":
    loop()"""
