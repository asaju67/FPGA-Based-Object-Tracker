
# -*- coding: utf-8 -*-

#%%
# import various libraries necessary to run your Python code
import time   # time related library
import sys,os    # system related library
import numpy as np
ok_sdk_loc = "C:\\Program Files\\Opal Kelly\\FrontPanelUSB\\API\\Python\\x64"
ok_dll_loc = "C:\\Program Files\\Opal Kelly\\FrontPanelUSB\\API\\lib\\x64"
from matplotlib.animation import FuncAnimation
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cv2


sys.path.append(ok_sdk_loc)   # add the path of the OK library
os.add_dll_directory(ok_dll_loc)

import ok     # OpalKelly library

#set registers 57-60, 69, 80, 83, 97, 98, 101-103, 107-110, 117
############################################
#okWireIn Addresses
#0x01: Reset_Counter
#0x02: write_in [7:0] ## data to be written 
#0x03: addr [7:0] ## W'/R + addr [6:0]
#0x04: frame_req -- unused
#0x05: PC_control (only for SPI)
############ for I2C ####################
#0x06: SAD
#0x07: SUB
#0x08: write_in_I2C
#0x09: flag_write
#0x0a: read1byte
#0x0b: direction
#0x0c: pulses
#0x0d: I2C_control (only to trigger I2C)

#okWireOut Addresses
#0x21: out [7:0] 
#0x22: pulse_count 
#0x23: out_x
#0x24: out_y
#0x25: out_z

#0xa0: FIFO
###########################################

############### for imu ###################
ASAD = 0b000000000011001
MSAD = 0b000000000011110
def convertAcc(value):
    value = value & 0xFFFF
    value = (value >> 4) & 0x0FFF
    if value & (1 << 11):
        value -= (1 << 12)
        
    return value * .001

def convertMag(value):
    value = (value & 0xFFFF) 
    if value & (1 << 15):
        value -= (1 << 16)
        
    return value * 0.002
def readMag():
    #read magnetometer
    time.sleep(0.03)
    PC_control = 1
    dev.SetWireInValue(0x0d, PC_control)
    dev.SetWireInValue(0x0a, 0) #read multiple bytes
    dev.SetWireInValue(0x06, 30) 
    dev.SetWireInValue(0x09, 0)
    dev.SetWireInValue(0x07, 0x03)
    dev.UpdateWireIns()
    
    
    dev.UpdateWireOuts()
    mx = convertMag(dev.GetWireOutValue(0x23))
    my = convertMag(dev.GetWireOutValue(0x24))
    mz = convertMag(dev.GetWireOutValue(0x25))
    
    print("Magnetism: " + "X: " + str(mx) +" Y: " + str(my) + " Z: " + str(mz))
    
    PC_control = 0
    dev.SetWireInValue(0x0d, PC_control)
    dev.UpdateWireIns()

    return mx, my, mz
    
def readAcc():
    time.sleep(0.03)

    PC_control = 1
    dev.SetWireInValue(0x0d, PC_control)
    dev.SetWireInValue(0x0a, 0) #read multiple bytes
    dev.SetWireInValue(0x06, 25) #set SAD
    dev.SetWireInValue(0x09, 0) #write flag
    dev.SetWireInValue(0x07, 0xA8) #SUB
    dev.UpdateWireIns()
    
    dev.UpdateWireOuts()
    
    
    ax = convertAcc(dev.GetWireOutValue(0x23))
    ay = convertAcc(dev.GetWireOutValue(0x24))
    az = convertAcc(dev.GetWireOutValue(0x25))
    
    # print(dev.GetWireOutValue(0x21), dev.GetWireOutValue(0x22), dev.GetWireOutValue(0x23))
    print("Acceleration: " + "X: " + str(ax) +" Y: " + str(ay) + " Z: " + str(az))
    
    
    PC_control = 0
    dev.SetWireInValue(0x0d, PC_control)
    dev.UpdateWireIns()
    
    return ax, ay, az

def configAcc():
    #Config Accelerometer
    ###############################################
    time.sleep(0.03)
    PC_control = 1
    dev.SetWireInValue(0x0d, PC_control)
    dev.SetWireInValue(0x06, 25) 
    dev.SetWireInValue(0x08, 0b00100111) #10Hz Data Rate
    dev.SetWireInValue(0x09, 1)
    dev.SetWireInValue(0x07, 0x20)
    dev.UpdateWireIns()  # Update the WireIns

    print("Configured Accelerometer")
    dev.UpdateWireOuts()
    

    #reading acceleromter
    PC_control = 0
    dev.SetWireInValue(0x0d, PC_control)
    dev.UpdateWireIns()

    #read back what was written to register
    # time.sleep(0.03)
    # PC_control = 1
    # dev.SetWireInValue(0x04, PC_control)
    # dev.SetWireInValue(0x02, 0)
    # dev.SetWireInValue(0x05, 1)
    # dev.UpdateWireIns()

    # PC_control = 0
    # dev.SetWireInValue(0x04, PC_control)
    # dev.UpdateWireIns()
    ##################################################

def configMag():
    #Config magnetometer
    ##################################################
    time.sleep(0.04)
    PC_control = 1
    dev.SetWireInValue(0x0d, PC_control)
    dev.SetWireInValue(0x06, 30) 
    dev.SetWireInValue(0x08, 0x10) #15 Hz Data Rate
    dev.SetWireInValue(0x09, 1)
    dev.SetWireInValue(0x07, 0x00)
    dev.UpdateWireIns()  # Update the WireIns

    PC_control = 0
    dev.SetWireInValue(0x0d, PC_control)
    dev.UpdateWireIns()

    # time.sleep(0.04)
    # #read back what was written to register
    # PC_control = 1
    # dev.SetWireInValue(0x04, PC_control)
    # dev.SetWireInValue(0x02, 0)
    # dev.SetWireInValue(0x05, 1)
    # dev.UpdateWireIns()

    # PC_control = 0
    # dev.SetWireInValue(0x04, PC_control)
    # dev.UpdateWireIns()

    time.sleep(0.04)
    PC_control = 1
    dev.SetWireInValue(0x0d, PC_control)
    dev.SetWireInValue(0x06, MSAD)
    dev.SetWireInValue(0x08, 0) #continuous conversion mode
    dev.SetWireInValue(0x09, 1)
    dev.SetWireInValue(0x07, 0x02)
    dev.UpdateWireIns()  # Update the WireIns

    PC_control = 0
    dev.SetWireInValue(0x0d, PC_control)
    dev.UpdateWireIns()

    # time.sleep(0.04)
    #read back what was written to register
    # PC_control = 1
    # dev.SetWireInValue(0x04, PC_control)
    # dev.SetWireInValue(0x02, 0)
    # dev.SetWireInValue(0x05, 1)
    # dev.UpdateWireIns()

    # PC_control = 0
    # dev.SetWireInValue(0x04, PC_control)
    # dev.UpdateWireIns()

    print("Configured Magnetometer")
    #################################################


###########################################

######## for camera #######################
def writeReg(write_in,addr):
    PC_control = 1
    dev.SetWireInValue(0x05, PC_control)
    dev.SetWireInValue(0x03, addr + 128) 
    dev.SetWireInValue(0x02, write_in) 
    dev.UpdateWireIns()
    
    # time.sleep(.001)
    dev.UpdateWireOuts()
    sent = dev.GetWireOutValue(0x22)
    PC_control = 0
    dev.SetWireInValue(0x05, PC_control)
    dev.UpdateWireIns()
    
    print("Wrote "+ str(sent) + " to address "+ str(addr))
    # time.sleep(.001)
def readReg( addr):
    PC_control = 1
    dev.SetWireInValue(0x05, PC_control)
    dev.SetWireInValue(0x03, addr) 
    dev.UpdateWireIns()
    
    # time.sleep(.001)
    dev.UpdateWireOuts()
    out = dev.GetWireOutValue(0x21)

    PC_control = 0
    dev.SetWireInValue(0x05, PC_control)
    dev.UpdateWireIns()
    print("Read value: " + str(out) + " from register: " + str(addr))
    # time.sleep(.001)
    return out

def configCam():#set registers 57-60, 69, 80, 83, 97, 98, 101-103, 107-110, 117
    writeReg( 3, 57)
    readReg( 57)
    writeReg( 44, 58)
    readReg(58)
    writeReg(240, 59)
    readReg(59)
    writeReg(10, 60)
    readReg(60)
    writeReg(9, 69)
    readReg(69)
    writeReg(2, 80)
    readReg(80)
    writeReg(187, 83)
    readReg(83)
    writeReg(240, 97)
    readReg(97)
    writeReg(10, 98)
    readReg( 98)
    writeReg( 98, 101)
    readReg(101)
    writeReg(34, 102)
    readReg(102)
    writeReg(64, 103)
    readReg(103)
    writeReg(110, 107)
    readReg(107)
    writeReg(91, 108)
    readReg(108)
    writeReg(82, 109)
    readReg(109)
    writeReg(80, 110)
    readReg(110)
    writeReg( 91, 117)
    readReg(117) 
    
    
    writeReg(2, 55)
    writeReg( 10, 43)
    print("----------------Finished Configuring Camera -----------------")
########################################################

def read_frame():
    print("----------------Read image data from the FPGA.----------------")
    buf = bytearray(315392)
    dev.SetWireInValue(0x01, 1)  # Trigger read
    dev.UpdateWireIns()
    time.sleep(0.0001)
    dev.SetWireInValue(0x01, 0)
    dev.UpdateWireIns()

    dev.ReadFromBlockPipeOut(0xa0, BLOCK_SIZE, buf)
    buf = np.array(buf, dtype = np.uint8)
    buf = buf[:-464]

    image = buf.reshape((486,648))
    cv2.imshow('Picture', image)
    cv2.waitKey(1) #change to cv2.waitKey(0) for pictures instead
    
    # return image[50][50]
    return image


############### Tracking ############

def activate_motor(direction, pulses):
    dev.SetWireInValue(0x0b, direction)
    dev.SetWireInValue(0x0c, pulses)
    # time.sleep(pulses/200) 

def control_motor(center_x):
    print("Control the motor based on the object's position relative to the image center.")
    image_center_x = IMAGE_WIDTH // 2
    pulses = 50  # Adjust based on motor's response

    if center_x < image_center_x:  # Object is on the left
        print("Turning motor LEFT")
        activate_motor(dev, LEFT, pulses)
    elif center_x > image_center_x:  # Object is on the right
        print("Turning motor RIGHT")
        activate_motor(dev, RIGHT, pulses)
    for i in range(pulses*2):
        readAcc(dev)
        readMag(dev)


def compute_center_of_mass(diff):
    print("Compute the center of mass of the object in the difference image.")
    # Threshold the difference image
    _, thresh = cv2.threshold(diff, THRESHOLD, 255, cv2.THRESH_BINARY)
    
    # Find coordinates of the pixels above the threshold
    coords = np.column_stack(np.where(thresh > 0))

    if len(coords) > 0:
        center_y, center_x = np.mean(coords, axis=0).astype(int)
        return center_x, center_y
    else:
        return None, None


#%% 
# Define FrontPanel device variable, open USB communication and
# load the bit file in the FPGA
dev = ok.okCFrontPanel()  # define a device for FrontPanel communication
SerialStatus=dev.OpenBySerial("")      # open USB communication with the OK board
# We will NOT load the bit file because it will be loaded using JTAG interface from Vivado

# Check if FrontPanel is initialized correctly and if the bit file is loaded.
# Otherwise terminate the program
print("----------------------------------------------------")
if SerialStatus == 0:
    print ("FrontPanel host interface was successfully initialized.")
else:    
    print ("FrontPanel host interface not detected. The error code number is:" + str(int(SerialStatus)))
    print("Exiting the program.")
    sys.exit ()


    



#%%
print("Testing Final Project")
IMAGE_WIDTH, IMAGE_HEIGHT = 648, 486
BLOCK_SIZE = 1024

##change according to what happens
LEFT = 0
RIGHT = 1
THRESHOLD = 30

direction = 1
pulses = 0
dev.SetWireInValue(0x0c, pulses)
dev.SetWireInValue(0x0b, direction)
dev.SetWireInValue(0x00, 0)  # Trigger read
dev.UpdateWireIns()

configCam()
time.sleep(0.05)
configAcc()
time.sleep(0.05)
configMag()
time.sleep(1)

start = time.time()
times = []



frames = 1000000
frame1 = read_frame()
for f in range(frames):
# while(True):
    frame2 = read_frame()
    diff = cv2.absdiff(frame1, frame2)
    times.append(time.time() - start)
    center_x, center_y = compute_center_of_mass(diff)

    if center_x is not None and center_y is not None:
        print(f"Center of Mass: X={center_x}, Y={center_y}")

        # Control the motor based on the center's position
        print("Control the motor based on the object's position relative to the image center.")
        image_center_x = IMAGE_WIDTH // 2
        pulses = 50  # Adjust based on motor's response

        if center_x < image_center_x:  # Object is on the left
            direction = LEFT
        elif center_x > image_center_x:  # Object is on the right
            direction = RIGHT
        dev.SetWireInValue(0x0c, pulses)
        dev.SetWireInValue(0x0b, direction)
        dev.UpdateWireIns()
        
        dev.SetWireInValue(0x0c, 0)
        dev.SetWireInValue(0x0b, direction)
        dev.UpdateWireIns()
        
        for i in range(pulses*2):
            readAcc(dev)
            readMag(dev)

    # Display the thresholded difference image (for debugging)
    cv2.imshow('Thresholded Difference', diff)
    frame1 = frame2
    dev.UpdateWireOuts()

    


cv2.destroyAllWindows()


#### Testing Motor and Acceleration alone
# pulse = 300
# direction = 1
# dev.SetWireInValue(0x0c, pulse)
# dev.SetWireInValue(0x0b, direction)
# dev.UpdateWireIns()
# dev.UpdateWireOuts()
# for i in range(10):
#     readAcc()
#     readMag()


# pulse = 0
# direction = 1
# dev.SetWireInValue(0x0b, direction)
# dev.SetWireInValue(0x0c, pulse)
# dev.UpdateWireIns()
# for i in range(10):
#     readAcc()
#     readMag()
    
# pulse = 300
# direction = 0
# dev.SetWireInValue(0x0c, pulse)
# dev.SetWireInValue(0x0b, direction)
# dev.UpdateWireIns()
# dev.UpdateWireOuts()
# for i in range(10):
#     readAcc()
#     readMag()


# pulse = 0
# direction = 0
# dev.SetWireInValue(0x0b, direction)
# dev.SetWireInValue(0x0c, pulse)
# dev.UpdateWireIns()
# for i in range(10):
#     readAcc()
#     readMag()

   
dev.Close