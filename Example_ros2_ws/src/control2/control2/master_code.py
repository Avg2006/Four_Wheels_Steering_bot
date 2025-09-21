#!/usr/bin/env python3

import rclpy as r
from rclpy.node import Node
import copy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
import queue
from operator import add
# from traversal2.msg import WheelRpm 
from rclpy.qos import QoSProfile, ReliabilityPolicy
from time import time, sleep
from geometry_msgs.msg import Twist

class Drive(Node):

    def __init__(self):
        super().__init__("drive_node")

        self.max_steer_pwm = 127              
        self.init_dir = [1] * 8                # Dont know what this is 
        self.state = False
        self.mode_up_button = 5
        self.mode_down_button = 4              # 0 -> 1 -> 2 -> 3 (up)
        self.fb_axis = 1                       # To move rover forward-back
        self.lr_axis = 3                       # To move rover left-right
        self.forward_btn = 3                   # To turn all wheels parallel to chassis
        self.parallel_btn = 1                  # To turn all wheels perpendicular to chassis
        self.rotinplace_btn = 2                         
        self.autonomous_btn = 0                # Autonomous button switches between manual and autonomous
        self.steer_islocked = True             # Checks if the steering has been unlocked
        self.steering_ctrl_unlocked = [0, 0]   # Will store the buttons which control when the steering is unlocked
        self.steering_ctrl_pwm = [0, 0]        # Will store the axis input when the steering is unlocked
        self.full_potential_islocked = True   # Checks if full potential steering is unlocked
        self.full_potential_pwm = [0, 0, 0, 0]      # Will store the axis input for each of the wheels when full potential is unlocked
        self.steer_unlock_axis = 5            # The joy axis that locks full potential when the steering is being toggled
        self.steer_samedir_axis = 3           # Tells which axis will be used for samedir motion
        self.steer_oppdir_axis = 4            # Tells which axis will be used for rotin place
        self.full_potential_unlock_axis = 2   # The joy axis that locks steering when full_potential is being toggled
        self.fl_wheel_axis = 1                # Axis that will give input for front left wheel
        self.fr_wheel_axis = 4                # Axis that will give input for front right wheel        
        self.bl_wheel_axis = 0                # Axis that will give input for back left wheel
        self.br_wheel_axis = 3
        self.drive_ctrl = [0, 0]              # Drive fb and lr axes
        self.steering_ctrl_locked = [0, 0, 0] # Gives configurations for steering (buttons)
        self.curve_opp_str = 0                # Stores input given by the fourth axis that moves the rover in a curve

        self.enc_data = [0.0] * 4             # Encoder data

        self.mode = 0                         # Goes from 0 to 4

        # Print parameters
        self.prints_per_iter = 1
        self.print_ctrl = self.prints_per_iter

        # Autonomous parameters
        self.crab_rotate = False
        self.autonomous_vel = 0                             # Velocity of wheels in autonomous mode
        self.autonomous_omega = 0                           # Omega of wheels in autonomous mode
        self.rotin = 0                                      # And need to understand what this is                
        self.state_init = [False, False, False]             # Need to understand this

        self.steering_complete = True
        #self.d_arr = [25,35,50,75,110]                     # Same as galileo drive multipliers
        self.drive_multipliers = [35,70,110,140,180]        # Drive multipliers
        self.s_arr = [self.max_steer_pwm] * 5               # No modes in steering       
        self.initial_enc_data = [0,0,0,0]
        self.initial_value_received = False
        self.kp_steer = 30                       
        self.qsize = 3                            
        self.vel_prev = queue.Queue(self.qsize)   # To implement ledc type control
        self.omega_prev = queue.Queue(self.qsize) # To implement ledc type control
        self.start_time = time()            # Start time to maintain 10s time limit for angle operations
        self.time_thresh = 10                     
        self.error_thresh = 2.0                   # Angle error threshold in deg

        self.rotinplace = False                   # Condition if rover is in rotin place alignment    

        self.in_steer = False
        self.final_angles = []

        # PWM message initialisation
        self.pwm_msg = Int32MultiArray()
        self.pwm_msg.layout = MultiArrayLayout()
        self.pwm_msg.layout.data_offset = 0
        self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
        self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
        self.pwm_msg.layout.dim[0].label = 'write'

        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, self.qos)
        self.enc_sub = self.create_subscription(Float32MultiArray, "/enc_auto", self.enc_callback, self.qos)
        self.rpm_sub = self.create_subscription(Twist, "/motion", self.autonomous_callback, self.qos)
        self.rot_sub = self.create_subscription(Int8, "/rot", self.rotinplace_callback, self.qos)

        self.pwm_pub = self.create_publisher(Int32MultiArray, "/motor_pwm", self.qos)
        # self.state_pub = self.create_publisher(Bool, "/state", self.qos)

        self.timer = self.create_timer(0.1, self.timer_callback)


    def joy_callback(self, joy: Joy):

        if not self.state:   # The rover is in manual mode  
            
            # This is the correct version, just want to see if I can do some jugaad
            if joy.buttons[self.mode_up_button]==1:
                sleep(1)
                if self.mode < 4:
                    self.mode += 1

            if joy.buttons[self.mode_down_button]==1:
                sleep(1)
                if self.mode > 0:
                    self.mode -= 1

            # self.mode += joy.buttons[self.mode_up_button] * (self.mode < 4)
            # self.mode -= joy.buttons[self.mode_down_button] * (self.mode > 0)

            match (self.steer_islocked, self.full_potential_islocked):
                case (True, True):
                    # This is the case when both the steering and full potential are locked
                    # In this case, only changing the wheels orientation is done

                    # self.steering_ctrl_locked stores all the buttons input for steering locked condition
                    self.steering_ctrl_locked = [
                        joy.buttons[self.forward_btn],    
                        joy.buttons[self.parallel_btn],
                        joy.buttons[self.rotinplace_btn]
                    ]
                    
                    # This variable is to get the fb and lr axes input from the controller
                    self.drive_ctrl = [joy.axes[self.fb_axis], -joy.axes[self.lr_axis]]
                    # There is a fourth axis apparently and this variable stores it
                    self.curve_opp_str = joy.axes[3] 

                case (False, True):
                    # In this case steering is unlocked 

                    # self.steering_ctrl_unlocked stores the buttons which are unlocked when steering is unlocked
                    self.steering_ctrl_unlocked = [
                        joy.buttons[self.forward_btn],
                        joy.buttons[self.parallel_btn]
                    ]
                    # self.steering_ctrl_pwm stores the inputs via the joystick axes during steering unlock condition
                    self.steering_ctrl_pwm = [
                        joy.axes[self.steer_samedir_axis],
                        joy.axes[self.steer_oppdir_axis]
                    ]

                case (True, False):
                    # In this full potential steering is unlocked

                    # self.full_potential_steering holds inputs for all the four wheels, when we want to achieve full potential steering
                    self.full_potential_pwm = [
                        joy.axes[self.fl_wheel_axis],
                        joy.axes[self.fr_wheel_axis],
                        joy.axes[self.bl_wheel_axis],
                        joy.axes[self.br_wheel_axis]
                    ]
            # Both cannot be unlocked at the same time, hence no case _ required

            # Need to understand what this is
            # Kind of get it but like why? question still remains
            if (joy.axes[self.steer_unlock_axis] == -1.0):  # Lock full potential when steering pwm is being toggled
                self.steer_islocked = not self.steer_islocked
                self.full_potential_islocked = True

            elif (joy.axes[self.full_potential_unlock_axis] == -1.0):   # Lock steering pwm when indiv control is being toggled
                self.full_potential_islocked = not self.full_potential_islocked
                self.steer_islocked = True

        # Changing mode of operation
        if joy.buttons[self.autonomous_btn]: 
            self.state = not self.state
            

    def enc_callback(self, msg: Float32MultiArray):
        data = msg.data
        # self.get_logger().info(len(data))
 
        self.enc_data[0] = data[0]   # Front Left
        self.enc_data[1] = data[1]  # Front Right
        self.enc_data[2] = data[2]   # Back Left
        self.enc_data[3] = data[3]   # Back Right


    def rotinplace_callback(self, msg: Int8):
        self.rotin = msg.data
        self.get_logger().info(self.rotin)


    def autonomous_callback(self, rpm: Twist):
        if self.state: 
            self.autonomous_vel = rpm.linear.x
            self.autonomous_omega = rpm.angular.z
            # self.crab_rotate = rpm.hb

    
    def autonomous_control(self):
        # Check first if the rover is in autonomous mode
        if self.state:
            # Just check when to print
            if self.print_ctrl==0:
                self.get_logger().warn("Rover in autonomous mode. Press A to enter manual mode.")

            ## Wonder what this part does ##
            self.steering_ctrl_locked[0] = 0
            self.steering_ctrl_locked[2] = 0

            match self.rotin:
                case 0:
                    self.state_init = [False, False, False]

                case 1:
                    if not self.state_init[0]:
                        self.steering_ctrl_locked[2] = 1
                        self.state_init[0] = True

                case 2:
                    if not self.state_init[1]:
                        self.state_init[0] = False
                        self.steering_ctrl_locked[0] = 1
                        self.state_init[1] = True

                    
    def steer(self, initial_angles, final_angles, mode):
        pwm = [0] * 4
        pwm_temp = [0] * 4
        # print("shaka laka boooommm")
        # This condition is to check if the wheels reached the final angle that it was told to go to
        # Within an error threshold

        # Encoder is relative
        relative_angle_conditions = [
            abs(self.enc_data[i] - initial_angles[i]) < abs(final_angles[i]) - self.error_thresh 
            for i in range(4)
        ]

        # Encoder is absolute
        absolute_angle_conditions = [
            abs(self.enc_data[i] - final_angles[i]) > self.error_thresh
            for i in range(4)
        ]

        angle_conditions = [relative_angle_conditions, absolute_angle_conditions]

        if any(angle_conditions[mode]) and (within_time := (time() - self.start_time)):
             # Printing only at certain intervals, to prevent the screen from being filed with data
            if(int(within_time) * 10 % 2 == 0):  
                print("Enc_data:- ", self.enc_data, end = "     ")
                print("Final angles:- ", list(map(add, initial_angles, final_angles)))
        
            '''
            pwm = [0,0,0,0]
            pwm_temp=[0,0,0,0]'''
            # Temporarily stores PWM stuff
            '''
            pwm_temp = [
                int(self.kp_steer * (final_angles[i] - ((self.enc_data[i] - initial_angles[i]) if mode == 0 else self.enc_data[i]))) 
                if angle_conditions[mode][i] else 0
                for i in range(4)
            ]'''

            # Finally constraining the PWM
            '''
            pwm = [
                    min(self.max_steer_pwm, pwm_temp[i]) 
                    if pwm_temp[i] >= 0 and angle_conditions[mode][i] else
                    max(-self.max_steer_pwm, pwm_temp[i])
                    if pwm_temp[i] < 0 and angle_conditions[mode][i] else
                    0 
                    for i in range(4) 
                ]'''
                
            for i in range(4):
                if (abs(self.enc_data[i]-final_angles[i])>self.error_thresh):
                    pwm_temp[i] = int(self.kp_steer*(final_angles[i]-self.enc_data[i]))
                    self.get_logger().info(f"{i} = {pwm_temp[i]}")
                    if pwm_temp[i]>=0:
                        pwm[i]=min(self.max_steer_pwm, pwm_temp[i])
                    else:
                        pwm[i] = max(-self.max_steer_pwm, pwm_temp[i])
                else:
                    pwm[i] = 0
            
            # PWM message
            self.pwm_msg.data = [
                0,0,0,0,
                pwm[0] * self.init_dir[4],
                pwm[1] * self.init_dir[5],
                pwm[2] * self.init_dir[6],
                pwm[3] * self.init_dir[7]
            ]
            # Publish the PWM message
            # self.create_rate(10).sleep()
            self.pwm_pub.publish(self.pwm_msg)
        else:
            # Visual
            print()
            print("***Steering Complete***")
            print()
            self.in_steer = False


    def steering(self):
        match (self.steer_islocked, self.full_potential_islocked):
            case (True, True):
                # This is the case when both the steering and full potential are locked
                # In this case, only changing the wheels orientation is done

                # If the forward button is pressed -> Align all the wheels forward
                # if self.steering_ctrl_locked[0] == 1:
                #     # Visualisation
                #     print()
                #     print("Rotating steering forward")
                #     print()

                #     self.steering_complete = False
                #     self.rotinplace = False
                #     self.start_time = time()
                #     self.steer([0,0,0,0],[0,0,0,0],1)

                # # Parallel button is pressed -> Align all the wheels perpendicular to the rover
                # elif self.steering_ctrl_locked[1] == 1:
                #     # Visualisation
                #     print()
                #     print("Rotating steering perpendicular to rover")
                #     print()

                #     self.steering_complete = False
                #     self.rotinplace = False
                #     self.start_time = time()
                #     self.steer([0,0,0,0],[90,90,90,90],1)

                # # Rotin button is pressed -> Align wheels for rotating in place
                # elif(self.steering_ctrl_locked[2] == 1):
                #     # Visualisation
                #     print()
                #     print("Rotating steering for in place rotation")
                #     print()
                #     # rotinplace must be made true only in manual mode
                #     # Then why isnt there a condition that takes self.autonomous_btn into account?
                #     self.rotinplace = True
                #     self.steering_complete = False
                #     self.start_time = time()
                #     self.steer([0,0,0,0],[55,-55,-55,55],1)

                if 1 in self.steering_ctrl_locked:
                    # Visualisation
                    index = self.steering_ctrl_locked.index(1)
                    visualisation_string_dict = {
                        0: "Rotating steering forward",
                        1: "Rotating steering perpendicular to rover",
                        2: "Rotating steering for in place rotation"
                    }

                    print()
                    print(visualisation_string_dict[index])
                    print()

                    self.steering_complete = False
                    self.rotinplace = (self.steering_ctrl_locked[2] == 1)
                    self.start_time = time()
                    final_angles_dict = {
                        0: [0, 0, 0, 0],
                        1: [90, 90, 90, 90],
                        2: [-55, 55, 55, -55]
                    }
                    self.final_angles = final_angles_dict[index]
                    self.in_steer = True
                    # self.steer(initial_angles=[0, 0, 0, 0], final_angles=self.final_angles, mode=1)
               
                elif (abs(self.curve_opp_str) > 0.2):
                    # self.curve_opp_str basically gives a curve motion so all the 
                    # wheels kind of align in order to achieve that
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time()

                    # Amount of curve needed multiplied with steering multiplier
                    temp = int(self.s_arr[self.mode] * self.curve_opp_str)

                    # PWM message
                    self.pwm_msg.data = [  
                        # Last four values of this message encode the omega for each wheel
                        0,0,0,0,
                        temp * self.init_dir[4],  
                        temp * self.init_dir[5], 
                        -temp * self.init_dir[6], 
                        -temp * self.init_dir[7]
                    ]

                    # Visualisation
                    print("Encoder angles:-", self.enc_data, end = "       ") 
                    print("Mode =", self.mode, end = "      ")
                    print("Curving with steering")

                else:
                    self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                    
        
            case (False, True):
                # This is the case when the steering is unlocked

                # Make a deep copy
                enc_data_new = copy.deepcopy(self.enc_data)
                
                # # Forward button is pressed -> Turn all wheels 45 deg from their current state in clockwise direction
                # if (self.steering_ctrl_unlocked[0] == 1):
                #     # Visualisation
                #     print()
                #     print("Turning steering clockwise by 45 deg")
                #     print()

                #     self.steering_complete = False
                #     self.rotinplace = False
                #     self.start_time = time()
                #     self.steer(enc_data_new, [45,45,45,45], 0) # Initial angle, final angle, mode=0 for relative

                # # Parallel button pressed -> Turn all wheels by 45 deg anticlockwise
                # elif (self.steering_ctrl_unlocked[1] == 1):
                #     # Visualisation
                #     print()
                #     print("Turning steering anti-clockwise by 45 deg")
                #     print()

                #     self.steering_complete = False
                #     self.rotinplace = False
                #     self.start_time = time()
                #     self.steer(enc_data_new, [-45,-45,-45,-45], 0) # initial angle, final angle, mode=0 for relative
                if 1 in self.steering_ctrl_unlocked:
                    index = self.steering_ctrl_unlocked.index(1)

                    visualisation_string_dict = {
                        0: "clockwise",
                        1: "anti-clockwise"
                    }
                    # Visualization
                    print()
                    print(f"Turning steering {visualisation_string_dict[index]} by 45 deg")
                    print()

                    self.final_angles = [45 if index == 0 else -45] * 4
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time()
                    self.in_steer = True
                    # self.steer(initial_angles=self.enc_data, final_angles=self.final_angles, mode=0)

                # If same dir axis is toggled
                elif (self.steering_ctrl_pwm[0] != 0 and abs(self.steering_ctrl_pwm[1]) < 0.2):     # Edit here to give operator threshold
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time()
                    temp = -int(self.s_arr[self.mode] * self.steering_ctrl_pwm[0])
                    # Create PWM message
                    self.pwm_msg.data = [
                        0,0,0,0,
                        temp * self.init_dir[4],
                        temp * self.init_dir[5],
                        temp * self.init_dir[6],
                        temp * self.init_dir[7]
                    ]

                    # Visualisation
                    print("Encoder angles:-", self.enc_data, end = "       ") 
                    print("Mode =", self.mode, end = "      ")
                    print("All wheels -> same direction.")

                # If opp dir axis is toggled -> Rotating in place with the velocity from axis
                elif (self.steering_ctrl_pwm[1] != 0 and abs(self.steering_ctrl_pwm[0]) < 0.2):
                    # Visualisation
                    print()
                    print("Moving with rot in place velocities")
                    print()

                    temp = int(self.max_steer_pwm * self.steering_ctrl_pwm[1])
                    self.pwm_msg.data = [
                        0,0,0,0,
                        temp * self.init_dir[4],
                       -temp * self.init_dir[5],
                       -temp * self.init_dir[6],
                        temp * self.init_dir[7]
                    ]
                    # Visualisation
                    print("Rotating in place with velocity =", temp)
                    print("Enc_angles:- ", self.enc_data)

                else:
                    self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                    if (self.print_ctrl == 0):    # Printing only at certain intervals, to prevent the screen from being filed with data
                        print("Steering is unlocked, lock it to perform drive.")

            case (True, False):
                # In this case, full_potential_steering is unlocked, so all the wheels can be controlled individually
                # fl_axis toggled -> front left wheel
                if (self.full_potential_pwm[0] != 0 and abs(self.full_potential_pwm[2]) < 0.2):   
                    temp = int(self.s_arr[self.mode] * self.full_potential_pwm[0])
                    self.pwm_msg.data = [
                        0,0,0,0,
                        temp * self.init_dir[4],
                        0,0,0
                    ]
                    # Visualisation
                    print("Encoder angles:-", self.enc_data, end = "       ")
                    print("Mode =", self.mode, end = "      ")
                    print("Moving front left wheel.")

                # fr_axis toggled -> front right wheel
                elif (self.full_potential_pwm[1] != 0 and abs(self.full_potential_pwm[3]) < 0.2):     
                    temp = int(self.s_arr[self.mode] * self.full_potential_pwm[1])
                    self.pwm_msg.data = [
                        0,0,0,0,0,
                        temp * self.init_dir[5],
                        0,0
                    ]
                    # Visualisation
                    print("Encoder angles:-", self.enc_data, end = "       ")
                    print("Mode =", self.mode, end = "      ")
                    print("Moving front right wheel.")
                    
                # bl_axis toggled -> back left wheel
                elif (self.full_potential_pwm[2] != 0 and abs(self.full_potential_pwm[0]) < 0.2): 
                    temp = int(self.s_arr[self.mode] * self.full_potential_pwm[2])
                    self.pwm_msg.data = [
                        0,0,0,0,0,0,
                        -temp * self.init_dir[6],
                        0
                        ]
                    print("Encoder angles:-", self.enc_data, end = "       ")
                    print("Mode =", self.mode, end = "      ")
                    print("Moving back left wheel.")

                # br_axis toggled -> back right wheel
                elif (self.full_potential_pwm[3] != 0 and abs(self.full_potential_pwm[1]) < 0.2):    
                    temp = int(self.s_arr[self.mode] * self.full_potential_pwm[3])
                    self.pwm_msg.data = [
                        0,0,0,0,0,0,0,
                        -temp * self.init_dir[7]
                    ]

                    # Visualisation
                    print("Encoder angles:-", self.enc_data, end = "       ")
                    print("Mode =", self.mode, end = "      ")
                    print("Moving back right wheel.")

                else:
                    self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                    if (self.print_ctrl == 0):    # Printing only at certain intervals, to prevent the screen from being filed with data
                        print("Individual steering control mode unlocked, lock it to perform drive.")

        # At the end, steering is complete
        self.steering_complete = True
        self.start_time = time()


    def drive(self):

        if self.steering_complete and self.steer_islocked and self.full_potential_islocked:
            # Steering is done, and both the types of steering are locked, only then we can drive the rover
            if self.rotinplace: # Wheels are in rotin place alignment
                # Get the velocity by multiplying the drive velocity multiplier with the axis output
                vel = self.autonomous_omega if self.state else self.drive_multipliers[self.mode] * self.drive_ctrl[1] 

                # PWM message
                self.pwm_msg.data = [
                    int(vel) * self.init_dir[0], 
                    -int(vel) * self.init_dir[1],
                    int(vel) * self.init_dir[2],
                    -int(vel) * self.init_dir[3], 
                    0,0,0,0]
                
                if self.print_ctrl == 0:    
                    # Printing only at certain intervals, to prevent the screen from being filled with data   
                    # Print_ctrl is being incremented in main() every time
                    print("Rotation speed =", int(vel))
                    
 
            else:  # Wheels are not aligned for rotin place

                # IDK if this is better but ternary is goated
                velocity = self.autonomous_vel if self.state else self.drive_multipliers[self.mode] * self.drive_ctrl[0]
                omega = self.autonomous_omega if self.state else self.drive_multipliers[self.mode] * self.drive_ctrl[1]
        
                avg_velocity = avg_omega = 0

                # Here queue is useful, FIFO right so we keep track of previous \
                # three velocities to smoothen the velocity that is actually given to the rover
                if self.vel_prev.full() and self.omega_prev.full():
                    # Calculate the average velocity and omega
                    avg_velocity = sum(self.vel_prev.queue) / self.qsize
                    avg_omega = sum(self.omega_prev.queue) / self.qsize

                    # Remove the first element
                    self.vel_prev.get() 
                    self.omega_prev.get()
                
                # Add new velocity to the queue
                self.vel_prev.put(velocity, True, 2)
                self.omega_prev.put(omega, True, 2)

                # Visualisation
                print("Velocity:", (avg_velocity // 0.001) / 1000) # the math part is to only print upto 2 decimals
                print("Omega: ", (avg_omega // 0.001) / 1000)
                print("Mode: ", self.mode)
                print()

                # PWM message
                # This v - w is done because it is apparently differential drive ig but 
                # the analogy is v - wr where r just disappeared but ok
                self.pwm_msg.data = [
                    int(avg_velocity + avg_omega) * self.init_dir[0],
                    int(avg_velocity - avg_omega) * self.init_dir[1],
                    int(avg_velocity + avg_omega) * self.init_dir[2],
                    int(avg_velocity - avg_omega) * self.init_dir[3],
                    0,0,0,0
                ]
                

            # PWM message initialisation
            self.pwm_msg.layout = MultiArrayLayout()
            self.pwm_msg.layout.data_offset = 0
            self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
            self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
            self.pwm_msg.layout.dim[0].label = 'write'
            self.pwm_pub.publish(self.pwm_msg)
        else:
            pass
    

    def timer_callback(self):
        if self.in_steer:
            print ("yooooooooooo")
            match (self.steer_islocked, self.full_potential_islocked):
                case (True, True):
                        self.steer(initial_angles=[0, 0, 0, 0], final_angles=self.final_angles, mode=1)
                case (False, True):
                        self.steer(initial_angles=self.enc_data, final_angles=self.final_angles, mode=0)
        else:
            self.get_logger().info(f"{self.steering_ctrl_locked}")

            if self.rotin != 0:
                self.get_logger().warn(f"{self.rotin}")

            if self.state:
                self.get_logger().info(f"auto vel is {self.autonomous_vel}")
                self.get_logger().info(f"auto omega is {self.autonomous_omega}")
            
            self.autonomous_control()
            self.steering()
            self.drive()
            self.pwm_pub.publish(self.pwm_msg)
            self.print_ctrl = (self.print_ctrl + 1) % self.prints_per_iter


def main(args=None):
    r.init(args=args)
    node = Drive()
    r.spin(node)
    r.shutdown()


if __name__ == "__main__":
    main()
    
