from math import floor, asin, degrees
from threading import Lock, Thread
from time import sleep

from datetime import date
from datetime import timedelta

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

import os
import re


BATTERY_HIGH = 0.95
BATTERY_LOW = 0.2  # when the robot will go charge
BATTERY_CRITICAL = 0.1  # when the robot will shutdown

#pkg_turtlebot4_heat_map = get_package_share_directory('turtlebot4_heat_map')
pkg_turtlebot4_heat_map = os.path.join(os.path.expanduser('~'),'cleaning_data')

class BatteryMonitor(Node):

    def __init__(self, lock):
        super().__init__('battery_monitor')

        self.lock = lock

        # Subscribe to the /battery_state topic
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

    # Callbacks
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


class WaypointNav(Node):
    def __init__(self, lock):
        super().__init__('waypoint_nav')

        self.lock = lock

        today = date.today()
        yesterday = str(today - timedelta(days=1))

        # Load the waypoint file, default = yesterday's plan
        self.declare_parameter('load_file', yesterday)

        self.filename = self.filename_callback()

    # Callbacks
    def filename_callback(self):
        with self.lock:
            filename = self.get_parameter('load_file').get_parameter_value().string_value
            
            return filename

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()

def main(args=None):
    rclpy.init(args=args)
    navigator = TurtleBot4Navigator()

    goal_pose = []
    isFinished = False

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)
    path_reader = WaypointNav(lock)

    battery_percent = None
    position_index = 0

    # Parallel processing nodes
    battery_thread = Thread(target=battery_monitor.thread_function, daemon=True)
    battery_thread.start()

    loader_thread = Thread(target=path_reader.thread_function, daemon=True)
    loader_thread.start()

    with lock:
        filename = path_reader.filename
        load_file = os.path.join(pkg_turtlebot4_heat_map,'saved_goals', filename)


    try:
        # Load the file into the robot if one was received
        with open(load_file, 'r') as f:
            for line in f:
                poses = re.findall(r'[-]?\d+\.\d+', line)
                
                # Convert string to Pose message type
                position = [float(poses[0]),float(poses[1])]
                rotation = degrees(asin(float(poses[-2]))*2)
                goal = navigator.getPoseStamped(position, rotation)
                goal_pose.append(goal)
        f.close()
        
        print('file has been loaded!')
        print('you can add more poses!')

    except:
        print('there is no file by that name!')

    # Create/Add new goal poses
    trajectories = navigator.createPath()

    # Add all the goal poses together
    if bool(goal_pose) == False:
        goal_pose = trajectories
    else:
        goal_pose = goal_pose + trajectories

    if len(goal_pose) == 0:
        navigator.error('No poses were given, exiting.')
        exit(0)

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.clearAllCostmaps()
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    while isFinished == False:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if (battery_percent is not None):
            navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

            # Check battery charge level
            if (battery_percent < BATTERY_CRITICAL):
                navigator.error('Battery critically low. Charge or power down')
                break
            elif (battery_percent < BATTERY_LOW):
                # Go near the dock
                navigator.info('Docking for charge')
                navigator.startToPose(navigator.getPoseStamped([-0.1, 0.0],
                                      TurtleBot4Directions.NORTH))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('Robot failed to dock')
                    break

                # Wait until charged
                navigator.info('Charging...')
                battery_percent_prev = 0
                while (battery_percent < BATTERY_HIGH):
                    sleep(15)
                    battery_percent_prev = floor(battery_percent*100)/100
                    with lock:
                        # Check battery status
                        battery_percent = battery_monitor.battery_percent

                    # Print charge level every time it increases a percent
                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

                # Undock
                navigator.undock()

            else:
                # Navigate to next position
                navigator.startToPose(goal_pose[position_index])

                # Navigate back to the dock
                position_index = position_index + 1
                if position_index >= len(goal_pose):
                    navigator.startToPose(navigator.getPoseStamped([-0.1, 0.0],
                                      TurtleBot4Directions.NORTH))
                    navigator.dock()
                    isFinished = True
    
    # Make a saved goals directory if it doesn't already exist
    os.makedirs(os.path.join(os.path.expanduser('~'), 'cleaning_data', 'saved_goals'), exist_ok=True)
    new_save = os.path.join(pkg_turtlebot4_heat_map,'saved_goals',str(date.today()))
    
    # Save the goals from the navigation goals
    with open(new_save, 'w') as f:
        for goal in goal_pose:
            f.write(str(goal)+'\n')
    f.close()

    # Shutdown the node
    path_reader.destroy_node()
    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 