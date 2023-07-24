#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import sys
import termios
import tty
from turtlesim.srv import SetPen
from robo_draw.srv import Motion, MotionRequest
import copy
import time  # Import time for tracking duration
import threading  # Import threading for non-blocking input
import queue  # Import queue to handle keyboard input data

LINEAR_VEL = 0.5  # Linear velocity for the TurtleBot (adjust as needed)
ANGULAR_VEL = 0.5  # Angular velocity for the TurtleBot (adjust as needed)
LINE_WIDTH = 1  # Width of the line drawn by the TurtleBot (adjust as needed)
KEY_BINDINGS = {
    'w': (1, 0),   # Move forward
    'a': (0, 1),   # Turn left
    's': (-1, 0),  # Move backward
    'd': (0, -1),  # Turn right
    ' ': (0, 0),    # Stop the robot when the spacebar is pressed
    'i': None,     # Invert pen
    'r': None,     # Reset the TurtleBot's position
    'p': None   # draw key
}

# Create a queue to handle keyboard input data
key_queue = queue.Queue()

def get_key():
    # Function to read a single character from stdin
    while True:
        file_descriptor = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_descriptor)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(file_descriptor, termios.TCSADRAIN, old_settings)
        key_queue.put(ch)

def set_pen(pen):
    # Function to invert the pen
    rospy.wait_for_service('turtle1/set_pen')
    try:
        set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
        set_pen(255, 255, 255, LINE_WIDTH, pen)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def reset_turtle():
    # Function to reset the TurtleBot's position
    rospy.wait_for_service('reset')
    try:
        reset = rospy.ServiceProxy('reset', Empty)
        reset()
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def item_to_motion(item):
    if type(item) is tuple:
        motion = MotionRequest(item[0], item[1], item[2])
        return motion
    raise TypeError("item must be a tuple of Twist and float and bool")

def draw(movements):
    print(movements)
    motions = [item_to_motion(item) for item in movements]

    try:
        draw_proxy = rospy.ServiceProxy('motion', Motion, persistent=True)
        for motion in motions:
            resp = draw_proxy(motion)
            print("Response:", resp)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def append_movement(twist_msg, twist_duration_list, isPenUp, start_time):
    end_time = time.time()
    duration = end_time - start_time
    twist_duration_list.append((copy.deepcopy(twist_msg), duration, isPenUp))
    print(f"Twist: {twist_msg}\nDuration: {duration}s\n")

def main():
    # Start the get_key function in a separate thread
    rospy.init_node('robo_draw_main', anonymous= True)  # Initialize the ROS node

    # Define the message publisher
    cmd_vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)

    # Define the Twist message to send velocity commands
    twist_msg = Twist()

    # List to store tuples of Twist messages and durations
    movement_instructions_list = []

    global penUp
    penUp = False
    set_pen(penUp)

    movement_instructions_list.append((copy.deepcopy(twist_msg), 0, penUp))

    import os
    os.system('cls' if os.name == 'nt' else 'clear')

    print("\n----------------------------")
    print("Use 'W', 'A', 'S', 'D' to move the TurtleBot.")
    print("Press 'Space' to stop it.")
    print("Press 'I' to invert the pen.")
    print("Press 'R' to reset the TurtleBot's position.")
    print("Press 'Ctrl + C' to exit.")
    print("----------------------------")

    last_key = None
    start_time = None
    last_update_time = None  # Time when last movement was appended
    try:
        while not rospy.is_shutdown():
            time.sleep(0.01)
            key = None
            while not key_queue.empty():
                key = key_queue.get()
            if key in KEY_BINDINGS:

                if key == 'i':
                    penUp = not penUp
                    set_pen(penUp)
                    movement_instructions_list.append((Twist(), 0, penUp))
                    continue
                elif key == 'p':
                    draw(movement_instructions_list)
                    movement_instructions_list = []
                    continue
                elif key == 'r':
                    reset_turtle()
                    return
                if key != last_key:
                    if last_key is not None:
                        append_movement(twist_msg, movement_instructions_list, penUp, start_time)
                    # if start_time is None or last_update_time == start_time:
                    start_time = time.time()  # Reset start_time whenever a new key is pressed
                    last_key = key
                linear_x, angular_z = KEY_BINDINGS[key]
                twist_msg.linear.x = linear_x * LINEAR_VEL
                twist_msg.angular.z = angular_z * ANGULAR_VEL
                cmd_vel_pub.publish(twist_msg)
                last_update_time = time.time()

            elif last_key is not None and time.time() - last_update_time > 1:
                append_movement(twist_msg, movement_instructions_list, penUp, start_time)
                last_key = None
                start_time = None
                last_update_time = None
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                cmd_vel_pub.publish(twist_msg)
            
            elif key is not None and key == '\x03':
                sys.exit(0)

            elif key is not None:
                print("Invalid key:", key)
                continue

    except rospy.ROSInterruptException:
        pass

    # Stop the robot before exiting
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmd_vel_pub.publish(twist_msg)

    # Add the last action if it was not added
    if start_time is not None:
        end_time = time.time()
        duration = end_time - start_time
        movement_instructions_list.append((twist_msg, duration))

if __name__ == '__main__':
    thread = threading.Thread(target=get_key, daemon=True)
    thread.start()

    while True:
        main()