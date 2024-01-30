import os
import re
import png
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from datetime import datetime
import functools
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

# Map an input range to an output range
def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min)*(out_max - out_min)//(in_max - in_min)+out_min

# Return the integer for the year, month and day from a datetime object
def extract_date():
    time  = str(datetime.now().time())
    date  = str(datetime.now().day)
    month = str(datetime.now().month)
    year  = str(datetime.now().year)

    return time, date, month, year

def extract_yaml(yamlf):
    # Open a file and return the lines where the resolution and origin are
    f = open(yamlf, 'r')
    resolution, origin = f.readlines()[2:4]

    # Get the resolution of the map
    resolution       = resolution.strip()
    resolution       = re.findall(r'\d+\.\d+',  resolution)
    resolution_float = [float(i) for i in resolution]
    
    # Get the origin of the map
    origin       = origin.strip()
    origin       = re.findall(r'[0-9]+\.*[0-9]*', origin)
    origin_float = [float(i) for i in origin]

    return resolution_float, origin_float

def initialize_new_map(pgmf):
    # Find the size of a map from the pgm file
    with open(pgmf,'rb') as f:
            size = str(f.readlines()[2:3][0])
            size = size[2:-3].split(" ")
            size = [int(i) for i in size]
            width, height = size
    map_matrix = np.array([],dtype=int)

    # Initialize a new matrix of the map with integers as its values
    with open(pgmf,'rb') as f:
        pgm_join = list(f.readlines()[4:])
        for line in pgm_join:
            for num in line:
                map_matrix = np.append(map_matrix,int(num))
        map_matrix = np.reshape(map_matrix,(height,width))

    return map_matrix, np.zeros_like(map_matrix)

def update_map(pose_matrix, resolution_float, origin_float, poseStamp, buffer):
    buff_temp = np.zeros_like(buffer)
    
    x_origin, y_origin, z_origin = origin_float
    
    # Return a list of only numbers from the PoseStamp message
    poses = re.findall(r'[-]?\d+\.\d+', str(poseStamp))
    
    # Isolate the numbers from the string into respective variables
    x_pose, y_pose = float(poses[0]),float(poses[1])
    #angle = degrees(asin(float(poses[5]))*2)

    # Map the map position to the earlier matrix
    x_map = int((x_origin + x_pose)/resolution_float[0])
    y_map = int(len(pose_matrix)-(y_origin + y_pose)/resolution_float[0]) #change operator on y_pose depending which direction is positive y
                                                                            #right now it's going up on original map

    # Set a footprint of 5x5 on the map for the robot
    for i in range(-2, 3):
        for j in range(-2, 3):
            check_pass = True
            # Remove consecutive repeat positions of the robot on the map
            for check in buffer:
                try:
                    # Check if any previous positions equals the current position
                    if any(check == [y_map+i, x_map+j]):
                        check_pass = False

                    # Update the temporary buffer
                    buff_temp  = np.insert(buff_temp, 0, [y_map+i, x_map+j], axis= 0)
                    buff_temp  = np.delete(buff_temp, -1, axis= 0)
                except IndexError:
                    print("out of map index range")

            # Increase the pose count of the robot in that cell
            if check_pass:        
                if pose_matrix[y_map+i, x_map+j] >= 30:
                    pose_matrix[y_map+i, x_map+j] = 30
                else:
                    pose_matrix[y_map+i, x_map+j] += 1
                
    buffer = np.array(buff_temp)

    return pose_matrix, buffer

def draw_png(map_matrix, pose_matrix, map_name, png_save):
    height,width = np.shape(pose_matrix)

    img = []
    for y in range(height):
        row = ()

        # Color the map blue if the robot has cleaned there, else leave the color as is
        for x in range(width):
            """  
            # Make a heat map based on the pose count of the robot against the maximum value of 30  
            if map_matrix[y][x] <= 10:
                row = row + (0, 0, 0)
            elif pose_matrix[y][x] < 1:
                row = row + (0, 180, 255)
            else:
                red_intensity = map_range(pose_matrix[y][x],1,100,100,200)
                row = row + (255,255-red_intensity,255-red_intensity)"""
            
            # Mark the map blue if the robot has been there once
            if map_matrix[y, x] <= 10 or pose_matrix[y, x] == 0:
                row = row + (map_matrix[y, x], map_matrix[y, x], map_matrix[y, x])
            else:
                row = row + (0, 120, 255)

        img.append(row)
    
    # Make the heat map and live map directory if it doesn't already exist
    os.makedirs(os.path.join(os.path.expanduser('~'), 'cleaning_data', map_name, 'heat_maps'), exist_ok= True)
    os.makedirs(os.path.join(os.path.expanduser('~'), 'cleaning_data', 'live_maps'), exist_ok= True)
    
    #Draw the png map
    with open(png_save, 'wb') as f:
        w = png.Writer(width, height, greyscale= False)
        w.write(f, img)

class MapGenerator(Node):
    def __init__(self, sub_cb_group, timer_cb_group):
        super().__init__('map_generator')

        # Initialize the resolution and origin of the map
        self.resolution = 0.0
        self.origin     = [0.0, 0.0, 0.0]
        self.buffer     = np.zeros((25,2),dtype= int)

        # Declare a parameter for the name of the map
        self.declare_parameter('map_name', 'open_day_map_obstacles')
        self.map_name = self.get_map_name()

        # Make a directory called live maps if it doesn't already exist
        os.makedirs(save_file, exist_ok=True)

        # Find the year, month and day of today for the save name of the png map and pose map
        time, day, month, year = extract_date()
        save_file = os.path.join(os.path.expanduser('~'), 'cleaning_data', self.map_name, 'pose_matrices', year, month, day)
        pose_save = os.path.join(save_file, time+'.txt')
        png_save =  os.path.join(os.path.expanduser('~'), 'cleaning_data', 'live_maps', str(datetime.now())+'.png')

        # Find the name of the pgm and yaml file using the name of the map
        pgmf  = os.path.join(os.path.expanduser('~'), 'maps', self.map_name+'.pgm')
        yamlf = os.path.join(os.path.expanduser('~'), 'maps', self.map_name+'.yaml')

        self.pgm_matrix, self.pose_matrix = initialize_new_map(pgmf)
        self.resolution, self.origin      = extract_yaml(yamlf)
        
        # Create a subscriber for the amcl pose and run parallel in a callback group
        self.subscriber                   = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10, callback_group= sub_cb_group)

        # Update the png map every 1 second and run parallel in a callback group
        timer_period_map = 1
        self.timer       = self.create_timer(timer_period_map, functools.partial(self.map_callback, pose_save, png_save), callback_group= timer_cb_group)
        
    #Callbacks
    def get_map_name(self):
        # Get the map name once when the node starts
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        return map_name

    def pose_callback(self,msg):
        # Update the pose map as soon as there is a change
        self.pose_matrix, self.buffer = update_map(self.pose_matrix, self.resolution, self.origin, msg, self.buffer)

    def map_callback(self, pose_save, map_save):
        # Make and save the png map and pose map
        draw_png(self.pgm_matrix, self.pose_matrix, self.map_name, map_save)
        
        with open(pose_save, "w") as g:
            for line in self.pose_matrix:
                for pose in line:
                    g.write(str(pose)+" ")
                g.write(str('\n'))

def main():
    rclpy.init()    

    # Make callback groups to run callbacks in parallel
    sub_cb_group   = MutuallyExclusiveCallbackGroup()
    timer_cb_group = MutuallyExclusiveCallbackGroup()
    
    map_generator = MapGenerator(sub_cb_group= sub_cb_group, timer_cb_group= timer_cb_group)
    executor      = MultiThreadedExecutor()
    executor.add_node(map_generator)

    # Happens when script is called in terminal
    print("")
    map_generator.get_logger().info("Spinning node")
    executor.spin()
    map_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
