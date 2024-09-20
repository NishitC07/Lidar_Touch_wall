# Run this code after Configuring the projector's Screen


import serial
import struct
import mouse as ms
import json
import math
from screeninfo import get_monitors
import numpy as np
import time 


with open('data.json','r') as f:
    data = json.load(f)

data1 = data[0]
data2 = data[2]
data3 = data[1]


orgi_x = data1["org_x"]
orgi_y = data1["org_y"]

UR_x = data2["upper_right_x"]
UR_y = data2["upper_right_y"]

BL_x = data3["Bottom_Left_x"]
BL_y = data3["Bottom_Left_y"]



pi = 22/7

monitor = get_monitors()
primary = monitor[0]

w = primary.width
h = primary.height

print(f"wid {w} hei {h}")

ai_w = int(UR_x - orgi_x)
ai_h = int(BL_y - orgi_y)

print(f"Width of AI:{ai_w}, Height of AI:{ai_h}")

x_pf = ai_w/w
y_pf = ai_h/h

dignl = np.sqrt(((UR_x)**2)+((BL_y)**2))
dignl = int(dignl) 
print(f'dignl is {dignl}')

max_angr = math.acos(((w * x_pf)/dignl)) 
max_ang = max_angr * (180/pi)

print(f'The max angle is {max_ang}')


def map_angle_to_value(angle):
    # Check if the angle is within the valid range
    if angle < 0 or angle > 90:
        return 0
    
    if 0 < angle < max_ang:
        x1, y1 = 0, UR_x
        x2, y2 = max_ang, dignl

        value = y1 + (y2 - y1) * (angle - x1) / (x2 - x1)
   
        return value 
    
    elif max_ang < angle < 90:
        x1, y1 = max_ang, dignl
        x2, y2 = 90, BL_y

        value = y1 + (y2 - y1) * (angle - x1) / (x2 - x1)
   
        return value

    else:
        return dignl



def extract_frames(data, header, bytes_to_read):
    frames = []
    current_frame = []
    header_encountered = False
    bytes_read_after_header = 0
    
    for byte in data:
        if byte == header:
            if header_encountered:
                if current_frame:
                    frames.append(current_frame)
                current_frame = []
                bytes_read_after_header = 0
            header_encountered = True
        if header_encountered:
            if bytes_read_after_header < bytes_to_read:
                current_frame.append(byte)
                bytes_read_after_header += 1
            else:
                header_encountered = False

    # Add the last frame if it exists
    if current_frame:
        frames.append(current_frame)
         
    return frames


def parse_lidar_data(data_packet):
    # Parse the Start Angle (2 bytes, little-endian)
    
    start_angle = struct.unpack('<H', data_packet[4:6])[0]
    start_angle_degrees = start_angle / 100.0
    
    # Parse the End Angle (2 bytes, little-endian)
    end_angle = struct.unpack('<H', data_packet[-5:-3])[0]
    end_angle_degrees = end_angle / 100.0
    
    
    # Parse the distances and intensities
    distances = []  
    object_data = data_packet[6:-5]  # Exclude the header, verLen, speed, angles, timestamp, and CRC check

    # Each measurement point consists of 2 bytes for distance and 1 byte for intensity
    for i in range(0, len(object_data), 3):
        if i + 2 < len(object_data):
            distance = struct.unpack('<H', object_data[i:i+2])[0]
            distances.append(distance)
    
    # Calculate angles for each point
    angle_increment = (end_angle_degrees - start_angle_degrees) / (len(distances) - 1)
    angles = [start_angle_degrees + i * angle_increment for i in range(len(distances))]

    # Return distances in meters and angles
    distances_meters = [d for d in distances]
    return distances_meters, angles

               
                
# Initialize the serial connection


def main():
    
    # Configure baudrate as per your requirement
    ser = serial.Serial("COM10", 921600, timeout=None)

    header = 0x54  # Header byte to identify the start of a frame
    bytes_to_read_after_header = 47

    try:
        while True:
            flag = True
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                
                frames = extract_frames(data, header, bytes_to_read_after_header)

                for frame in frames:
                    if len(frame) == 47:
                        # Convert frame to hexadecimal string
                        hex_frame = ' '.join(f"{byte:02X}" for byte in frame)
                        
                        distances, angles = parse_lidar_data(bytes.fromhex(hex_frame))
                        for distance, angle in zip(distances, angles):
                         

                         distance = int(distance)
                         value = map_angle_to_value(angle)


                         if 1 < angle < 89 and flag:

                          
                          if 0 < distance < value : 
                           print(f'the distance is {distance}')
                           print(f'The angle is {angle}')
                           print(f"The value is {value}")                         
                        
                           angle_r = math.radians(angle)    

                           x_dis = distance * math.cos(angle_r)
                           y_dis = distance * math.sin(angle_r)

                           x_dis = x_dis - orgi_x
                           y_dis = y_dis - orgi_y

                           X = int(x_dis/x_pf) 
                           Y = int(y_dis/y_pf)


                           flag = False
                           ms.release()
                           ms.move(X,Y,duration=0.00000000000000000000000000000000000000000000000000000000000000001)\
                            
                           ms.click()
                           
                           
                           
                        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
