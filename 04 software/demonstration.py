import os, time, struct
from socket import *
import numpy as np


LOCAL_IP   = '192.168.1.102'  # check it
LOCAL_PORT = 6060

MISH_IP   = '192.168.1.103'
MISH_PORT = 2333

BIAS      = np.deg2rad(0)  # [rad]
AMPLITUDE = np.deg2rad(30)  # [rad]
FREQUENCY = 0.5  # [Hz]


if __name__ == '__main__':
    mish_socket = socket(AF_INET, SOCK_DGRAM)
    mish_socket.bind(('', LOCAL_PORT))

    mish_socket.settimeout(0.1)

    start_timestamp = time.perf_counter()
    while(True):
        recv_data, recv_addr = mish_socket.recvfrom(1024)
        recv_data = list(struct.unpack('3i3f3f3i', recv_data))
        print(recv_data)

        theta = np.clip(np.sin(2 * np.pi * (time.perf_counter() - start_timestamp) * FREQUENCY) * AMPLITUDE + BIAS, -np.deg2rad(60), np.deg2rad(60))  # [rad]
        send_data = struct.pack('ffffff', theta, theta, theta, 0, 0, 0)
        mish_socket.sendto(bytes(send_data), (MISH_IP, MISH_PORT))
        
        while(time.perf_counter() - start_timestamp < 0.02):
            pass
