import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


from geometry_msgs.msg import Pose
import time
from rclpy.executors import SingleThreadedExecutor
import threading
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from std_msgs.msg import Empty



class PupperController(Node):

    def __init__(self):
        super().__init__('pupper_controller')
        self.create_subscription(String,'/voice_cmd',self.voice_cmd_callback,10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.pose_subscriber = self.create_subscription(Pose, "/body_pose", self.pose_callback, 10)
        self.x = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.pose = Pose()
        
        self.thread_executor = ThreadPoolExecutor(max_workers=1)

        self.move_executor = SingleThreadedExecutor()
        move_thread = threading.Thread(target=self.move_executor.spin)
        move_thread.start()
        print('ROSGPT Robo Dog Controller Started. Waiting for input commands ...')
    
    def pose_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.theta = msg.orientation
        self.pose = msg

    def stop(self):
        print('Stopping the pupper ...')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        self.velocity_publisher.publish(twist_msg)
        print('The Robot has stopped...')

    #this callback represents the ROSGPTParser. It takes a JSON, parses it, and converts it to a ROS 2 command
    def voice_cmd_callback(self, msg):
        #print(msg.data)
        try:
            cmd = json.loads(msg.data)
            cmd = json.loads(cmd['json']) #we only consider the pure json message. cmd['text'] contains a mix of text and json
            print('JSON command received: \n',cmd,'\n')

            if cmd['action'] == 'move':
                linear_speed = cmd['params'].get('linear_speed', 0.2)
                distance = cmd['params'].get('distance', 1.0)
                direction = cmd['params'].get('direction', "forward")

                print(f'linear_speed: {linear_speed}, distance: {distance}, direction: {direction}')

                self.thread_executor.submit(self.move, linear_speed, distance, direction)

            elif cmd['action'] == 'stop':
                self.thread_executor.submit(self.stop)


            elif cmd['action'] == 'rotate':
                angular_speed = cmd['params'].get('angular_speed', 0.2)
                angle = cmd['params'].get('angle', 1.0)
                direction = cmd['params'].get('direction', "clockwise")
                body_part = cmd['params'].get('body_part', "head")

                print(f'angular_speed: {angular_speed}, angle: {angle}, direction: {direction}')

                self.thread_executor.submit(self.rotate, angular_speed, angle, direction, body_part)

 
        except json.JSONDecodeError:
            print('[json.JSONDecodeError] Invalid or empty JSON string received:', msg.data)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))   


    def get_distance(self, start, destination):
        return math.sqrt(
            ((destination.position.x - start.position.x) ** 2) +
            ((destination.position.y - start.position.y) ** 2) +
            ((destination.position.z - start.position.z) ** 2)
        )

    def move(self, linear_speed, distance, direction): 
        print(f'Start moving the pupper {direction} at {linear_speed} m/s for a distance of {distance} meters')

        if abs(linear_speed) > 1.0:
            print('[ERROR]: The speed in any direction must be lower than 1.0!')
            return -1
        
        
        linear_vector = Vector3()

        try: 
            if direction == "forward":
                linear_vector.x = linear_speed
                linear_vector.y = 0.0
                linear_vector.z = 0.0

            elif direction == "backward":
                linear_vector.x = -linear_speed
                linear_vector.y = 0.0
                linear_vector.z = 0.0

        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))

        twist_msg = Twist()
        twist_msg.linear = linear_vector

        try:
            # Set the start time
            start_time = time.time()

            while time.time() - start_time < 5:  # Loop for 5 seconds
                self.velocity_publisher.publish(twist_msg)
                self.move_executor.spin_once(timeout_sec=0.5)


        except Exception as e:

            print('[Exception] An unexpected error occurred:', str(e))
        # Stop the movement after 5 seconds
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0
        self.velocity_publisher.publish(twist_msg)

        print("Stopping the pupper ...")


    def rotate(self, angular_speed, distance, direction, body_part): 
        print(f'Start moving the pupper {direction} at {angular_speed} m/s for a distance of {distance} meters')

        if abs(angular_speed) > 1.0:
            print('[ERROR]: The speed in any direction must be lower than 1.0!')
            return -1
        
        
        angular_vector = Vector3()
        linear_vector = Vector3()

        try: 
            if body_part == "head" and direction == "clockwise":
                '''
                linear:
                x: 0.5
                y: 0.0
                z: 0.0
                angular:
                x: 0.0
                y: 0.0
                z: -1.0
                ---
                '''

                linear_vector.x = 0.5
                angular_vector.x = 0.0
                angular_vector.y = 0.0
                angular_vector.z = -angular_speed

            elif body_part == "head" and direction == "anticlockwise":
                '''
                linear:
                x: 0.5
                y: 0.0
                z: 0.0
                angular:
                x: 0.0
                y: 0.0
                z: 1.0
                ---
                '''

                linear_vector.x = 0.5
                angular_vector.x = 0.0
                angular_vector.y = 0.0
                angular_vector.z = angular_speed


            elif body_part == "torso" and direction == "clockwise":
                '''
                linear:
                x: -0.5
                y: 0.0
                z: 0.0
                angular:
                x: 0.0
                y: 0.0
                z: -1.0
                ---
                ---
                '''
                linear_vector.x = -0.5
                angular_vector.x = 0.0
                angular_vector.y = 0.0
                angular_vector.z = -angular_speed

            elif body_part == "torso" and direction == "anticlockwise":
                '''
                linear:
                x: -0.5
                y: 0.0
                z: 0.0
                angular:
                x: 0.0
                y: 0.0
                z: 1.0
                ---
                '''
                linear_vector.x = -0.5
                angular_vector.x = 0.0
                angular_vector.y = 0.0
                angular_vector.z = angular_speed
                

            linear_vector.x = 0.0
            linear_vector.y = 0.0



        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))

        twist_msg = Twist()
        twist_msg.linear = linear_vector
        twist_msg.angular = angular_vector

        try:
            start_pose = copy.copy(self.pose)
            # Set the start time
            start_time = time.time()
            print('start_pose: ', start_pose)
            print('current_pose: ', self.pose)
            while time.time() - start_time < 5:  # Loop for 5 seconds
                self.velocity_publisher.publish(twist_msg)
                self.move_executor.spin_once(timeout_sec=0.5)


        except Exception as e:

            print('[Exception] An unexpected error occurred:', str(e))
        # Stop the movement after 5 seconds
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0
        self.velocity_publisher.publish(twist_msg)

        print("Stopping the pupper ...")

    
def main(args=None):
    rclpy.init(args=args)
    node = PupperController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
