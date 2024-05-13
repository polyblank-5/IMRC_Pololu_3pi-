from J_maths_module import *
#from J_state_estimate_ticks import *
from J_robot import *
from math import atan2, sqrt, sin, cos
import time
import json

#directs a trajectory from states & actions found in a json file
#returns true as long as it is controlling. When the goal is reached/
#all the actions have been made, returns False
def control_from_json(rob, states, actions, gains, t_start_traj) -> bool :
    state, t_ns = rob.state_estimator.state_estimate()
    t = t_ns - t_start_traj #time.ticks_diff(t_us, t_start_traj)
    t *= (10**-9) #convert to seconds
    x,y,theta = state
    #define gains
    K_x, K_y, K_theta = gains
    
    #get the index corresponding to the time we are at
    #for actions we round down, for states we round up (=round down + 1)
    #we round to the nearest decimal since the timestep is 0.1s
    index_action = int(round_down(t,1) * 10)
    index_state = index_action + 1
    print(t)
    print(f"action i {index_action} ; state i {index_state}")
    
    #get desired state and velocities
    if index_action >= len(actions):
        print("no more action left : goal should be reached")
        rob.motors.off()
        return False
    
    #get desired state and velocities
    x_d, y_d, theta_d = states[index_state]
    v_d, omega_d = actions[index_action]

    
    #compute error
    x_e = (x_d-x)*cos(theta) + (y_d - y)*sin(theta)
    y_e = -(x_d - x)*sin(theta) + (y_d - y)*cos(theta)
    theta_e = theta_d - theta
    
    
    #compute unicycle-model control variables (forwards speed and rotational speed)
    v_ctrl = v_d*cos(theta_e) + K_x * x_e
    omega_ctrl = omega_d + v_d*(K_y*y_e + K_theta*sin(theta_e)) + K_theta*theta_e
    
    #for logging
    rob.state_estimator.last_v_ctrl = v_ctrl
    rob.state_estimator.last_omega_ctrl = omega_ctrl
    
    #transform unicycle-model variables v_ctrl and omega_ctrl to differential-drive
    #model control variables (angular speed of wheels) 
    u_L, u_R = rob.trsfm_ctrl_outputs(v_ctrl, omega_ctrl)
    #transform [rad/s] speed to a value the motors can understand (0-6000)
    u_L, u_R = rob.angular_speed_to_motor_speed(u_L), rob.angular_speed_to_motor_speed(u_R)
    rob.motors.set_speeds(u_L, u_R)
    return True


    
rob = Robot()
traj = rob.choose_traj()
print(traj)

with open(traj) as f:
    data = json.load(f)
states = data["result"][0]['states']
ctrl_actions = data["result"][0]["actions"]
print(len(states), " ", states)
print(len(ctrl_actions), " ", ctrl_actions)
time.sleep(0.5)
gains = rob.set_gains()
print(gains)
time.sleep(1)
rob.state_estimator.start(True)
print("starting controler")
counter = 0
controlling=True
#time where the trajectory starts, with t=0.0 being the moment where the robot is initialized
#this isn't great coding I should change it later
t_start_traj = time.time_ns() - rob.state_estimator.starttime
print("start " + str(t_start_traj))
rob.state_estimator.display_state()
while controlling :
    controlling = control_from_json(rob, states, ctrl_actions, gains, t_start_traj)
    counter += 1
    if counter % 25 == 0:
        rob.state_estimator.display_state()
    time.sleep(0.005)
rob.motors.off()
rob.state_estimator.display_state()
rob.state_estimator.start(False)
rob.state_estimator.write_states_to_json(gains=gains, traj=traj)
print("finished")
    

