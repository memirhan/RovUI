from pySerialTransfer import pySerialTransfer as txfer
import time

link = txfer.SerialTransfer('/dev/ttyACM0', 9600)
link.open()
time.sleep(2)

while True:
    sendSize = 0
    roliCam = [90, 50]  # örnek açı ve dim
    sendSize = link.tx_obj(roliCam, start_pos=sendSize)
    link.send(sendSize)
    time.sleep(1)