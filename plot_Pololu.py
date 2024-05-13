import json
  

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import numpy as np
import os





def plot_trajectories():

    with open("curve.json") as f:
        data = json.load(f)
    states = data["result"][0]['states']
    ctrl_actions = data["result"][0]["actions"]  

    x_pos = np.array([state[0] for state in states])
    y_pos = np.array([state[1] for state in states])


    with plt.xkcd():
        # Based on "Stove Ownership" from XKCD by Randall Munroe
        # https://xkcd.com/418/

        # fig,ax = plt.subplots()
        fig = plt.figure()
        ax = fig.add_axes((0.1, 0.2, 0.8, 0.7))  #i don't know what this line actually does
        ax.spines[['top', 'right']].set_visible(True)   #True for a full rectangle around the graph, false for only x-y axis
        # ax.set_xticks(np.arange)
        # ax.set_yticks([])
        # ax.set_ylim([-30, 10])
        ax.plot(x_pos,y_pos)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        fig.text(
            0.5, 0.05,
            '"Pololu trajectory" in xkcd graph',
            ha='center')

        plt.show()

    with plt.xkcd():
        ###real###

        with open("run12_46_29.json") as f:
            realdata = json.load(f)
            realstates = realdata["states"]
        
        realx = np.array([state[0] for state in states])
        realy = np.array([state[1] for state in states])
        fig = plt.figure()
        ax = fig.add_axes((0.1, 0.2, 0.8, 0.7))  #i don't know what this line actually does
        ax.spines[['top', 'right']].set_visible(True)   #True for a full rectangle around the graph, false for only x-y axis
        ax.plot(realx,realy)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        fig.text(
            0.5, 0.05,
            '"real trajectory" in xkcd graph',
            ha='center')

        plt.show()


    
def plot_individual(jsonfile, traj):
    # trajectory_file = "/media/julien/MicroPython/trajectories/curve.json"
    trajectory_file = "/media/julien/MicroPython/trajectories/" + traj
    with open(trajectory_file) as f:
        data = json.load(f)
    states = data["result"][0]['states']
    ctrl_actions = data["result"][0]["actions"]  
    print(len(ctrl_actions))
    #no action is given for the last timestep since robot is assumed to have finished trajectory and stopped
    #for the dimensions to match we will thus append the "rest action" v_ctrl = 0, omega_ctrl=0 to the arrays
    ctrl_actions.append([0,0])
    print(len(ctrl_actions))

    x_pos_desired = np.array([state[0] for state in states])
    y_pos_desired = np.array([state[1] for state in states])
    theta_desired = np.array([state[2] for state in states])
    #the planner has a timestep of 0.1s, so we need a time array like such [0, 0.1, 0.2, ..... len(states)*0.1]
    time = np.arange(0, len(states)) * 0.1 #create an array of the same size as desired states with 0.1 interva between each value
    v_desired = np.array([action[0] for action in ctrl_actions])
    omega_desired = np.array([action[1] for action in ctrl_actions])

    print(len(time), len(x_pos_desired))
   
    with open(jsonfile) as f:
        realdata = json.load(f)
        realstates = realdata["states"]
        real_ctrl_actions = realdata["actions"]
        #real_ctrl_actions.append([0,0])
    realx = np.array([state[0] for state in realstates])
    realy = np.array([state[1] for state in realstates])
    realtheta = np.array([state[2] for state in realstates])
    realtime = np.array([state[3] for state in realstates])
    realtime = realtime * (10**(-9))
    realv = np.array([action[0] for action in real_ctrl_actions])
    realomega = np.array([action[1] for action in real_ctrl_actions])

    #calculate the arrays for the angular speeds of the wheels
    desired_uL , desired_uR = np.zeros_like(v_desired),np.zeros_like(v_desired)
    real_uL, real_uR = np.zeros_like(realv), np.zeros_like(realv)
    #assert len(desired_uL) == len(real_uL), "Problem in the array lengths"
    L = 0.0862 #interwheel distance of 3pi+ (approximately measured with a ruler)
    r = 0.016   #radius of the wheel
    for i in range(len(v_desired)):
        desired_uL[i] = (2*v_desired[i] - L*omega_desired[i]) / (2*r)
        desired_uR[i] = (2*v_desired[i] + L*omega_desired[i]) / (2*r)
    for i in range(len(realv)):
        real_uL[i] = (2*realv[i] - L*realomega[i]) / (2*r)
        real_uR[i] = (2*realv[i] - L*realomega[i]) / (2*r)




    #create title page
    text = 'Pololu 3pi+ trajectory test (straight line)'
    trajectory = "trajectory: " + trajectory_file.replace("/media/julien/MicroPython/trajectories/", "")
    gains = realdata["gains"]
    title_text_gains = f"K_x = {gains[0]}   K_y = {gains[1]}    K_theta = {gains[2]}"
    # jsonfile_name = jsonfile[jsonfile.find("run"):]
    # run_name = f"file name : {jsonfile_name}"
    run_name = f"file name : {jsonfile}"

    title_text = text + "\n" + trajectory + "\n" + run_name + "\n" + title_text_gains + "\n"
    fig = plt.figure(figsize=(5,8))
    fig.text(0.1, 0.1, title_text, size=11)
    traj_name = trajectory.replace("trajectory: ", "")
    traj_name = traj_name.replace(".json", "")
    pdf_name = f"{traj_name}_{gains[0]}_{gains[1]}_{gains[2]}.pdf"
    pdf_name = file_guard(pdf_name)
    pdf_pages = PdfPages(pdf_name)
    pdf_pages.savefig(fig)


     #create plots
    fig, ax = plt.subplots()
    ax.plot(time, x_pos_desired, label='Desired x position', linestyle="--", linewidth=1, zorder=10)
    ax.plot(realtime, realx, label='Recorded')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('x position [m]')  
    ax.set_title("Trajectory x")
    
    ax.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax.minorticks_on()
    fig.tight_layout(pad = 4)
    fig.legend()
    
    pdf_pages.savefig(fig) 
        
    fig2, ax2 = plt.subplots()
    ax2.plot(time, y_pos_desired, label='Desired y position', linestyle="--", linewidth=1, zorder=10)
    ax2.plot(realtime, realy, label='Recorded')
    ax2.set_xlabel('time [s]')
    ax2.set_ylabel('y position [m]')  
    ax2.set_title("Trajectory y")
    ax2.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax2.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax2.minorticks_on()
    fig2.tight_layout(pad = 4) 
    fig2.legend()
    pdf_pages.savefig(fig2) 
        

    fig3, ax3 = plt.subplots()
    ax3.plot(time, theta_desired, label='Desired theta', linestyle="--", linewidth=1, zorder=10)
    ax3.plot(realtime, realtheta, label='Recorded')
    ax3.set_xlabel('time [s]')
    ax3.set_ylabel('theta [rad]')   
    ax3.set_title("Angle theta")
    ax3.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax3.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax3.minorticks_on()
    fig3.tight_layout(pad = 4)
    fig3.legend()
    pdf_pages.savefig(fig3) 

    # fig4, ax4 = plt.subplots()
    # ax4.plot(time, self.euclidian_dist)
    # ax4.set_xlabel('time [s]')
    # ax4.set_ylabel('Euclidean distance [m]')
    # ax4.set_title('Deviation between ideal and recorded trajectories')
    # fig4.tight_layout(pad=4)
    # ax4.grid(which='major', color='#DDDDDD', linewidth=0.8)
    # ax4.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    # ax4.minorticks_on()
    # pdf_pages.savefig(fig4)


    fig5,ax5 = plt.subplots()
    ax5.plot(x_pos_desired, y_pos_desired, label='Ideal trajectory', linestyle="--", linewidth=1, zorder=10)
    ax5.plot(realx, realy, label='Recorded trajectory')
    ax5.set_xlabel('x [m]')
    ax5.set_ylabel('y [m]')
    ax5.set_title('2D visualization')
    fig5.tight_layout(pad=4)
    ax5.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax5.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax5.minorticks_on()
    pdf_pages.savefig(fig5)

    fig6, ax6 = plt.subplots()
    ax6.plot(time, v_desired, label='Desired', linestyle="--", linewidth=1, zorder=10)
    ax6.plot(realtime, realv, label='Recorded')
    ax6.set_xlabel('time [s]')
    ax6.set_ylabel('v [m/s]')   
    ax6.set_title("Translational velocity v_ctrl (unicycle model)")
    ax6.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax6.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax6.minorticks_on()
    fig6.tight_layout(pad = 4)
    fig6.legend()
    pdf_pages.savefig(fig6) 

    fig7, ax7 = plt.subplots()
    ax7.plot(time, omega_desired, label='Desired', linestyle="--", linewidth=1, zorder=10)
    ax7.plot(realtime, realomega, label='Recorded')
    ax7.set_xlabel('time [s]')
    ax7.set_ylabel('omega [rad/s]')   
    ax7.set_title("Angular velocity omega_ctrl (unicycle model)")
    ax7.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax7.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax7.minorticks_on()
    fig7.tight_layout(pad = 4)
    fig7.legend()
    pdf_pages.savefig(fig7) 




    fig8, (ax8, ax9) = plt.subplots(1, 2)
    fig8.suptitle('Left and right wheel angular velocities uL & uR')
    ax8.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax8.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax8.plot(time, desired_uL, label='Desired', linestyle="--", linewidth=1, zorder=10)
    ax8.plot(realtime, real_uL, label='Recorded', linestyle="--", linewidth=1, zorder=10)
    ax9.grid(which='major', color='#DDDDDD', linewidth=0.8)
    ax9.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    ax9.plot(time, desired_uR, linestyle="--", linewidth=1, zorder=10)
    ax9.plot(realtime, real_uR, linestyle="--", linewidth=1, zorder=10)
    fig8.legend()
    pdf_pages.savefig(fig8) 


    pdf_pages.close()
    plt.close('all')
    print(f"created {pdf_name}")



def plot_all(path):
    # os.chdir("/")
    os.chdir(path)
    runs = os.listdir()
    if runs == []:
        print("logs directory is empty. If it should contain logs but they're not showing up, try to restart the 3pi+")
    for run in runs:
        print(f"plotting {run}")
        run_namestart = run[:3]
        traj = None
        if run_namestart == "lin":
            traj = "line.json"
        elif run_namestart == "rot":
            traj = "rotation.json"
        elif run_namestart == "crv":
            traj = "curve.json"
        else:
            print(f"{run} was not plotted since no reference trajectory was provided")
            continue

        plot_individual(run, traj=traj)



def file_guard(pdf_name):
    '''Checks whether the pdf name already exists in the working directory. If yes, returns the filename with a B at the end. ("report.pdf" -> "reportB.pdf")
    Does this check recursively (if report.pdf exists, checks if reportB.pdf exists, if yes, checks if reportBB.pdf exists, etc until it finds a free name)'''
    filenames = os.listdir()
    if pdf_name in filenames:
        index = pdf_name.find(".pdf")
        pdf_name = pdf_name[:index] + "B" + pdf_name[index:]
        return file_guard(pdf_name)
    return pdf_name

if __name__ == "__main__":
    # plot_individual("/media/julien/MicroPython/logs/run0_0_23.json")
    plot_all("/media/julien/MicroPython/logs")
    # print(file_guard("1_1_1.pdf"))