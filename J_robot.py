import time   #is this precise enough or should I use some time module of the 3pi+ #answer : it wasn't. we need ticks
from math import sin, cos, pi
from J_maths_module import *
from machine import Timer
import json
from J_state_estimator import State_Estimator

from pololu_3pi_2040_robot import robot as three_pi_rob

class Robot():
    def __init__(self):
        #specific to the 3pi+ robot
        self.r = 0.016      #wheel radius [m]
        self.L = 0.0862     #distance between wheels [m] (this is an approx)
        self.encoders = three_pi_rob.Encoders()
        self.motors = three_pi_rob.Motors()
        self.display = three_pi_rob.Display()
        self.max_speed = 70*pi #max angular speed of the Hyper motors [rad/s]
        
        self.state_estimator = State_Estimator(robot=self)
        
        
        
        
    
    #transform "unicycle model" ctrl variables v_d
    #and omega_d in useable ctrl outputs, ie angular
    #speeds for the wheel motors
    #input : translational speed v, rotational speed omega
    #output : angular speed of the left and right wheels u_L and u_R
    def trsfm_ctrl_outputs(self, v, omega):
        u_L = (2*v - self.L*omega) / (2*self.r)
        u_R = (2*v + self.L*omega) / (2*self.r)
        return u_L, u_R
  
    #the 3pi+ motors have 6000 speed levels, with the highest
    #corresponding to 70*Pi [rad/s]. This method translates an angular
    #speed to the corresponding value (rounded down) in motor speed
    #input : speed in [rad/s]
    #output : motor speed level from 0 to 6000
    def angular_speed_to_motor_speed(self, speed):
        increment = self.max_speed/6000 #increment is 7pi/600, approx 0.0367 [rad/s]
        level = speed // increment
        if speed > 6000 :
            print("angular speed higher than maximum speed of the 3pi+ Hyper")
            level = 6000
        return level
  
    #print info about the current state on the display
    def display_state(self):
        x,y,rad = self.state_estimator.state
        theta = self.state_estimator.theta_easy
        t = self.state_estimator.last_estimation - self.state_estimator.starttime
        to_display = [f"x: {x}", f"y: {y}", f"deg: {theta}", f"rad: {rad} ",f"t {t / (10**9)}"]
        displaylist(to_display)

    
    def set_gains(self) -> tuple:
        gains = [1,1,1]
        gain_nr = 0 #0 = Kx, 1 = Ky, 2 = Ktheta
        which_gain = {0:'K_x', 1:'K_y', 2:'K_theta'}
        power = 0
        increment = 0.1
        
        display = three_pi_rob.Display()
        button_a = three_pi_rob.ButtonA()
        button_b = three_pi_rob.ButtonB()
        button_c = three_pi_rob.ButtonC()
        bump_sensors = three_pi_rob.BumpSensors()
        bump_sensors.calibrate()
        while True:
            bump_sensors.read()
            
            if bump_sensors.right.check():
                gain_nr = (gain_nr + 1) % 3
                
            display.fill(0)
            display.text(f"Tuning {which_gain[gain_nr]}", 0, 0)
            display.text("value: " + str(gains[gain_nr]), 0, 8)
            display.text(f"increment {increment}", 0,16)
            display.text(f"A: +   B: - ",0,24)
            display.text(f"C: incrementx10",0,32)
            display.text(f"Bumpers:", 24,40)
            display.text(f"Left: finish",0,48)
            display.text(f"Right: change K",0,56)

            if button_a.check():
                gains[gain_nr] += 1 * increment
            if button_b.check():
                gains[gain_nr] -= 1 * increment
            if button_c.check():
                power = ((power + 1) % 5)   #that way we can go 10^-1 to 10^3
                increment = 10**(power-1)
                print(power, " ", increment)
            if bump_sensors.left.check():
                print("gains set")
                return tuple(gains)

            display.show()
            
            
    def choose_traj(self):
        display = three_pi_rob.Display()
        bump_sensors = three_pi_rob.BumpSensors()
        button_a = three_pi_rob.ButtonA()
        bump_sensors.calibrate()
        
        index = 0
        traj_list = ["line","rotation","curve"]
        while True:
            bump_sensors.read()
            if bump_sensors.right.check():
                index = (index + 1) % 3
                display.fill(0)
                print(index)
            if button_a.check(): 
                print(index)
                return "/trajectories/" + traj_list[index] + ".json"
            display.text(f"Choose traj", 0, 0)
            display.text("StraightLine", 0, 16)
            display.text(f"rotation", 0,24)
            display.text(f"Curve",0,32)  
            display.text(f"R_Bumper: change", 0,48)
            display.text(f"A : select",0,56)
            display.text("<-", 100, (index+2)*8)
            display.show()
            
        

    
    #a method which displays elements of a list line by line. Since the display can only handle 7 lines, 
    #all elements of the list after the 7th position will not be displayed 
    def displaylist(self, textlist):
        display = three_pi_rob.Display()
        for i, msg in enumerate(textlist):
            display.text(msg, 0, i*8)
        display.show()
    
    
    
    
    
#a few lines to test the basic functionality of odometry state estimation   
#NB "if name == main" will only run if you run it from a terminal. Selecting the program from the screen of the 3pi+ will NOT run this code. If you want to run it from the control screen of the 3pi+,
#comment out if name==main and unindent to run it as a script. But be careful because this will then also run every time this file is imported. So keep the if clause there unless testing smth specific

