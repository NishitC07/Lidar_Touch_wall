#Use this Code to Configure the Projector's Screen
#To configure the projector's screen hold object first on upper left corner then lower left corner then upper #right corner watch data.json file to know if your input is taken or not



import serial
import struct
import sys
import math
import json
from concurrent.futures import ThreadPoolExecutor
import time
import os
from screeninfo import get_monitors

max_width = int(input("Enter Maximum width range for lidar "))
max_height = int(input("Enter Maximum height range for lidar "))


xlis = []
ylis = []

def map_angle_to_value(angle):
    # Check if the angle is within the valid range
    if angle < 0 or angle > 90 :
        return 0
    
    if 0 < angle < 90 :
        x1, y1 = 0, max_width
        x2, y2 = 90, max_height

        value = y1 + (y2 - y1) * (angle - x1) / (x2 - x1)
   
        return value

def write_data_to_file(data, filename):
    # Check if the file exists
    if os.path.exists(filename):
        # Read the existing data
        with open(filename, 'r') as f:
            existing_data = json.load(f)
            if len(existing_data) == 3:
                existing_data = []
    
    else:
        existing_data = []  # Start a new list if the file doesn't exist

    existing_data.append(data)  # Append new data
    with open(filename, 'w') as f:
        json.dump(existing_data, f, indent=4)

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

    if current_frame:
        frames.append(current_frame)
         
    return frames

def parse_lidar_data(data_packet):
    start_angle = struct.unpack('<H', data_packet[4:6])[0]
    start_angle_degrees = start_angle / 100.0
    end_angle = struct.unpack('<H', data_packet[-5:-3])[0]
    end_angle_degrees = end_angle / 100.0
    
    distances = []
    object_data = data_packet[6:-5]

    for i in range(0, len(object_data), 3):
        if i + 2 < len(object_data):
            distance = struct.unpack('<H', object_data[i:i+2])[0]
            distances.append(distance)
    
    angle_increment = (end_angle_degrees - start_angle_degrees) / (len(distances) - 1)
    angles = [start_angle_degrees + i * angle_increment for i in range(len(distances))]

    distances_meters = [d for d in distances]
    return distances_meters, angles

def countdown(seconds):
    try:
        
        while seconds:
            mins, secs = divmod(seconds, 60)
            timer = f'{mins:02}:{secs:02}'
            print(f'\r{timer}')
            sys.stdout.flush()
            time.sleep(1)
            seconds -= 1
        Flag = True
        print('\r00:00\n')
    except KeyboardInterrupt:
        print("\nCountdown interrupted.")

    return Flag
        

def main():
    ser = serial.Serial("COM10", 921600, timeout=None)
    header = 0x54
    bytes_to_read_after_header = 47

    with ThreadPoolExecutor() as Exp:
        result = Exp.submit(countdown,10)
        result2 = Exp.submit(countdown,15)
        result3 = Exp.submit(countdown,25)
        while True:
            Flag = False
            Flag2 = False
            Flag3 = False
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                frames = extract_frames(data, header, bytes_to_read_after_header)

                for frame in frames:
                    
                    if len(frame) == 47:
                        hex_frame = ' '.join(f"{byte:02X}" for byte in frame)
                        distances, angles = parse_lidar_data(bytes.fromhex(hex_frame))
                        
                        for distance, angle in zip(distances, angles):
                            distance = int(distance)
                            value = map_angle_to_value(angle)
                            if 1 < angle < 89 and distance < value:
                                

                                angle_r = math.radians(angle)    
                                x_dis = distance * math.cos(angle_r)
                                y_dis = distance * math.sin(angle_r)
                                if x_dis == 0.0:
                                    continue

                               
                                if result.done():
                                    Flag = result.result()
                                    
                                if Flag == True and len(xlis) == 0:
                                    Flag = False
                                    xlis.append(x_dis)
                                    ylis.append(y_dis)

                                    data = {"org_x":x_dis,
                                            "org_y":y_dis}
                                    write_data_to_file(data, "data.json")
                                        

                                if result2.done():

                                     
                                    Flag2 = result2.result()
                                    
                                if Flag2 == True and len(xlis) == 1:
                                    Flag = False
                                    xlis.append(x_dis)
                                    ylis.append(y_dis)

                                    data = {"Bottom_Left_x":x_dis,
                                            "Bottom_Left_y":y_dis}
                                    write_data_to_file(data, "data.json")
                                        
  

                                if result3.done():
                                    Flag3 = result3.result()
                                    
                                if Flag3 == True and len(xlis) == 2:
                                    # print(Flag)
                                    xlis.append(x_dis)
                                    ylis.append(y_dis)
   
                                    data = {"upper_right_x":x_dis,
                                            "upper_right_y":y_dis}
                                    write_data_to_file(data, "data.json")
                                    break

                        if Flag3:
                            break
            if Flag3:
                break           
                 
                                    

                            


            









if __name__ == "__main__":
    main()
