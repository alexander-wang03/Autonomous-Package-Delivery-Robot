##!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray, UInt32
import numpy as np
import colorsys
import statistics
import matplotlib.pyplot as plt

# Controller Class
class Controller(object):
    def __init__(self):

        # Publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.index = 0
        self.twist = Twist()
        self.line_filter = []

        # PID Controller Variables
        self.integral = 0
        self.derivative = 0
        self.lasterror = 0
        self.angular_v = 0
        self.angular_filter = []

    def camera_callback(self, msg):
        """
        Callback for line index.
        """

        self.index = msg.data

        # Line Filter
        if len(self.line_filter) > 5:
            self.line_filter.pop(0)
        self.line_filter.append(self.index)

        # Set detected line index based on Line Filter
        self.index = float(statistics.mean(self.line_filter))

    def follow_the_line(self):
        """
        Line-following PID Controller.
        """

        desired = 320
        actual = self.index  # Current sensor reading

        error = desired - actual

        k_p = 1/340 
        k_i = 4e-6 
        k_d = 1e-2 

        # PID calculations
        self.integral = self.integral + error
        self.derivative = error - self.lasterror

        # Limit integral error to +/-20000
        self.integral = max(min(self.integral, 20000), -20000)
        
        # Calculate Angular Velocity
        if abs(error) >= 50:
            self.angular_v = ((k_p*error) + (k_i*self.integral) + (k_d*self.derivative))*1.55
        else: # Remove Integral Term if error is low
            self.angular_v = ((k_p*error) + (k_d*self.derivative))*1.55
        
        # Limit Angular Velocity to +/-0.4
        self.angular_v = max(min(self.angular_v, 0.4), -0.4)

        # Setting the velocities
        self.twist.linear.x = 0.04
        self.twist.angular.z = self.angular_v

        # Store the last set of Angular Velocities
        if len(self.angular_filter) > 400:
            self.angular_filter.pop(0)
        self.angular_filter.append(self.angular_v)

        # Publishing the command
        self.cmd_pub.publish(self.twist)

        # Storing the last error for the next derivative calculation
        self.lasterror = error
    
    def straight_line(self):
        """
        When detecting an Office, move in a straight line.
        """

        self.twist.linear.x = 0.05
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
    
    def correct_left(self):
        """
        When entering a new Office, detect if a 90 degree turn was made with angular_filter.
        If so, provide left turning correction scaled to the sharpness of the 90 degree turn.
        """

        av = float(statistics.mean(self.angular_filter)) # Find the Average of the last set of angular velocities
        print(av)

        # If a 90 degree turn was performed recently, provide additional left turn correction to Turtlebot
        if av > 0.05: 

            for i in range(8): # Provide correction
                self.twist.linear.x = 0.05 
                self.twist.angular.z = min(1.5 * av * 10, 2.25) # Scale correction by sharpness of the turn detected. Cap correction to 2.25
                self.cmd_pub.publish(self.twist)

                rate.sleep()

            self.angular_filter = [0] * 400 # Reset stored angular velocity values
    
    def deliver_package(self):
        """
        Perform package delivery function at desired office
        """

        # Go straight for a duration into office
        for i in range(60):
            self.straight_line()
    
            rate.sleep()
        
        # Turn left 90 degrees
        for i in range(60):
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5708
            self.cmd_pub.publish(self.twist)

            rate.sleep()

        # Pause to Deliver Package
        for i in range(60):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_pub.publish(self.twist)

            rate.sleep()

        # Turn right 90 degrees
        for i in range(60):
            self.twist.linear.x = 0
            self.twist.angular.z = -1.5708
            self.cmd_pub.publish(self.twist)

            rate.sleep()


# Bayesian Localizer Class
class BayesLoc:
    def __init__(self, p0, colour_map):

        # Sensor Data Variables
        self.colour_sub = rospy.Subscriber("mean_img_rgb", Float64MultiArray, self.colour_callback)
        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)

        # PID Controller
        self.controller = Controller()

        # Bayesian Localization Variables
        self.num_states = len(p0)
        self.colour_map = colour_map
        self.probability = p0 # Current State Variable
        self.state_prediction = np.zeros(self.num_states) 

        # Colour Variables
        self.rgb = [] # RGB Value
        self.hsv = [] # HSV Value
        self.colour_filter = [] # Colour Filter (for Controller)
        self.cur_colour = None  # Most recent measured colour (for Controller)

        self.colour_measurments = [] # Colour Filter (for Localization)

        self.colour_codes = [
            [244, 160, 131],
            [163, 183, 163],
            [186, 178, 197],
            [190, 179, 158],
            [150, 150, 150]
        ] # Reference colour RGBs (for Measurement Model)

    def line_callback(self, msg):
        """
        Callback for line index.
        """
        
        self.controller.camera_callback(msg)

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """

        self.rgb = np.array(msg.data)  # [r, g, b]
        self.hsv = colorsys.rgb_to_hsv(self.rgb[0]/255.0, self.rgb[1]/255.0, self.rgb[2]/255.0)
        self.hsv = [self.hsv[0]*360.0, self.hsv[1]*100.0, self.hsv[2]*100.0]
        # print(self.hsv)
        self.estimate_color()


    def estimate_color(self):
        """
        Estimate currently observied office colour based on HSV values
        """

        if 0 <= self.hsv[0] < 20:          # Use saturation to differentiate between orange or line (since they look similar in hue due to warm coloured laboratory lights)
            if 20 <= self.hsv[1] < 80:
                color = 0
            else:
                color = 4
        elif 20 <= self.hsv[0] < 25:          # orange 5-25
            color = 0
        elif 25 <= self.hsv[0] <= 55:       # yellow 25-35
            color = 3
        elif 85 <= self.hsv[0] <= 165:     # green 125-165
            color = 1
        elif 220 <= self.hsv[0] <= 310:     # blue 270-290
            color = 2
        else:                               # line
            color = 4

        # Colour Filter (for PID Controller)
        if len(self.colour_filter) > 5:
            self.colour_filter.pop(0)
        self.colour_filter.append(color)

        # Colour Measurements Filter (for Localization)
        if len(self.colour_measurments) > 100:
            self.colour_measurments.pop(0)
        self.colour_measurments.append(color)
        
        # Set current detected colour based on Colour Filter
        self.cur_colour = int(statistics.median(self.colour_filter) + 0.5)

    def state_model(self):
        """
        State model: p(x_{k+1} | x_k, u)
        """

        state_predict = np.roll(self.probability, 1)

        return state_predict

    def measurement_model(self):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """

        measurement_model = np.zeros(len(self.colour_codes)-1)

        for i in range(len(self.colour_codes)-1):
            measurement_model[i] = 1/math.sqrt((self.colour_codes[i][0] - self.rgb[0])**2 + (self.colour_codes[i][1] - self.rgb[1])**2 + (self.colour_codes[i][2] - self.rgb[2])**2)

        return measurement_model

    def state_predict(self):
        """
        update self.state_prediction with the predicted probability of being at each
        state (office)
        """

        self.state_prediction = self.state_model()

    def state_update(self):
        """
        update self.probabilities with the probability of being at each state
        """

        measurement_model = self.measurement_model()

        for i in range(len(self.colour_map)):
            self.probability[i] = self.state_prediction[i] * measurement_model[self.colour_map[i]]
        
        # Normalize
        prob_sum = sum(self.probability)
        for i in range(len(self.colour_map)):
            self.probability[i] = self.probability[i] / prob_sum


if __name__ == "__main__":

    # Initialize Localization Parameters 
    colour_map = [3, 1, 2, 0, 0, 1, 2, 0, 3, 1, 2] # (0: orange, 1: green, 2: blue, 3: yellow, 4: line) 
    p0 = [1/11] * 11 # Initial probability of being at a given office is uniform
    offices = [0, 3, 6  ] # Office delivery locations


    rospy.init_node("final_project")
    localizer = BayesLoc(p0, colour_map)
    rospy.sleep(0.5)
    rate = rospy.Rate(60) # Increased from 30

    # Initialize Control Parameters
    last_state = ""
    color_timer = rospy.Time.now().to_sec() - 2
    color_registered = False

    # Initialize Plotting Parameters
    total_steps = 0
    steps_per_plot = 6 # Number of steps to visualize in each plot
    office_indices = np.arange(2, 13) 
    plot_colour_map = {0: 'orange', 1: 'green', 2: 'blue', 3: 'yellow'}  # RGB values for the colours
    plot_prob = []

    while not rospy.is_shutdown():
        if localizer.cur_colour == 4: # Line

            # PID Line Following
            localizer.controller.follow_the_line()

            # Set State to Line
            last_state = "line"

        else: # Colour

            # If we are entering a new Colour from a Line
            if last_state == "line":
                localizer.controller.correct_left() # Apply correction if necessary
                color_registered = False

            # Straight Line
            localizer.controller.straight_line()

            # State Predict and Update for new Office Location
            if rospy.Time.now().to_sec() - color_timer > 3 and not color_registered and all(el in [0,1,2,3] for el in localizer.colour_measurments):

                # State Predict and State Update
                localizer.state_predict()
                localizer.state_update()
          
                rospy.loginfo(localizer.probability)
                rospy.loginfo(localizer.probability.index(max(localizer.probability)))
                
                # If confident of state, deliver package at desired office
                if max(localizer.probability) > 0.7:
                    curr_office = localizer.probability.index(max(localizer.probability)) # Currently perceived office index

                    if curr_office in offices:
                        localizer.controller.deliver_package() # Deliver package
                        offices.pop(offices.index(curr_office)) # Remove office from list to deliver after delivering
                
                # Store state to plot
                total_steps += 1
                plot_prob += [np.copy(localizer.probability)]

                color_registered = True 
                color_timer = rospy.Time.now().to_sec() # Reset timer

            # Set State to Colour
            last_state = "colour"

        rate.sleep()

    rospy.loginfo("finished!")

# Visualization of results
for plot_index in range(0, total_steps, steps_per_plot):
    plt.figure(figsize=(10, 8))
    for i in range(steps_per_plot):
        step = plot_index + i
        if step >= total_steps:
            break
        plt.subplot(steps_per_plot, 1, i + 1)

        # Assigning colors to bars based on colour_map
        bar_colours = [plot_colour_map[colour] for colour in colour_map]

        plt.bar(office_indices, plot_prob[step], color=bar_colours)
        plt.xticks(office_indices)  # Ensure each office number is a tick mark
        plt.ylim(0, 1)  # Set y-axis limit for better comparison
        plt.axhline(y=0.7, color='r', linestyle='-') # Add Confidence Threshold Line
        plt.ylabel('Probability')
        plt.title(f'Step {step + 1} Probability Distribution')
    
    plt.xlabel('Office Number')  # Only set x-label for the bottom subplot
    plt.tight_layout()  # Adjust layout for neatness
    plt.show()