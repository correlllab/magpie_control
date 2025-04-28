import struct
import socket
import numpy as np
import multiprocessing, ctypes
from time import sleep
import random

RESPONS_SZ = 36
FORCE_DIV  =  10000.0 # -------------- Default Force  divide value
TORQUE_DIV = 100000.0 # -------------- Default Torque divide value


class OptoForceCmd:
    """ Container class for OptoForce commands """
    
    ## Datagrams ##
    DG_cmd = struct.Struct('! 2H I') #- 2x uint8, 1x uint32, Big-endian (Network)
    DG_res = struct.Struct('! 3I 6i') # 3x uint32, 6x int32, Big-endian (Network)
    COMMANDS = { 
            'set_speed_100' : DG_cmd.pack( 0x1234 , #- Header
                                           0x0082 , #- Set speed
                                               10 ), # Speed /* 1000 / SPEED = Speed in Hz */
            'set_speed_50' : DG_cmd.pack( 0x1234 , #- Header
                                          0x0082 , #- Set speed
                                              20 ), # Speed /* 1000 / SPEED = Speed in Hz */
            'set_filter_0' : DG_cmd.pack( 0x1234 , #- Header
                                          0x0081 , #- Set filter
                                               0 ), # No filter
            'set_bias_0' : DG_cmd.pack( 0x1234 , #- Header
                                        0x0042 , #- Set bias
                                             0 ), # No bias
            'set_bias_1' : DG_cmd.pack( 0x1234 , #- Header
                                        0x0042 , #- Set bias
                                             1 ), # Yes bias
            'send_10' : DG_cmd.pack( 0x1234 , #- Header
                                     0x0002 , #- Data Request
                                         10 ), # Number of samples
            'send_01' : DG_cmd.pack( 0x1234 , #- Header
                                     0x0002 , #- Data Request
                                          1 ), # Number of samples
            'send_02' : DG_cmd.pack( 0x1234 , #- Header
                                     0x0002 , #- Data Request
                                          2 ), # Number of samples
            'stop_data' : DG_cmd.pack( 0x1234 , #- Header
                                       0x0000 , #- Data Request
                                           10 ), # Number of samples
        }
                
    @staticmethod
    def unpack_response( res ):
        """ Unpack the response into a python list """
        return list( OptoForceCmd.DG_res.unpack( res )[3:] ) # Trim off the header


class OptoForce: 
    def __init__(self, ip_address: str = "192.168.0.5", port: int = 49152, poll_rate=50):
        self.sensorAddr = (ip_address, port)
        self.sock_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd = OptoForceCmd()
        if poll_rate not in [50, 100]:
            raise ValueError("Poll rate must be 50 or 100")
        self.poll_rate_cmd = self.cmd.COMMANDS[f'set_speed_{poll_rate}']
        
    def connect(self):
        """ Connect to the sensor """
        self.sock_r.settimeout(5)
        self.sock_r.connect(self.sensorAddr)
        self.sock_r.setblocking(0)
        self.prime_sensor()

    def prime_sensor(self):
        """ Prime the sensor for data collection """
        self.send_datagram(self.poll_rate_cmd)
        self.send_datagram(self.cmd.COMMANDS['set_filter_0'])
        self.send_datagram(self.cmd.COMMANDS['set_bias_1'])

    def send_datagram(self, commandBytes, wait_s = 0.020):
        """ Send the command over the socket and wait a bit """
        self.sock_r.send(commandBytes)
        sleep(wait_s)

    def recv_datum(self):
        """ Attempt to recieve a datum from the socket """
        rtnDat  = []
        dataLen = 0
        for i in range(5):
            if dataLen != RESPONS_SZ:
                rtnDat  = []
                dataLen = 0
                try:
                    self.send_datagram(self.cmd.COMMANDS['send_01'])
                    data, _ = self.sock_r.recvfrom(RESPONS_SZ) # buffer size is 1024 bytes
                    dataLen = len(data)
                    if dataLen == RESPONS_SZ:
                        rtnDat = self.cmd.unpack_response(data)
                        for i in range(3):
                            rtnDat[i  ] /= FORCE_DIV
                            rtnDat[i+3] /= TORQUE_DIV
                except BlockingIOError as err:
                    print("Error reading from OptoForce!")
                    # if the read fails, clear the buffer and try again
                    self.send_datagram(self.cmd.COMMANDS['stop_data'])
            else:
                break
            return rtnDat

    def close(self):
        """ Close the socket """
        self.sock_r.close()

