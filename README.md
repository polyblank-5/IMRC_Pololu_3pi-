This repository contains code for the 3pi+ 2040 Pololu Ground Robot. It has a simple state estimator based on odometry readings and a simple differential drive controler. To make gain tuning easier, you can choose a trajectory from a selection of three(straight line, pure rotation, slightly wavy diagonal)
and adjust the gains (Kx, Ky, Ktheta) you want to use directly with the robot buttons. (These two options are enabled by default). The state estimator logs some data of interest and there is also a script which plots this into neatly readable graphs.
This code was written for the IMRC Lab of TU Berlin. It is also based on the Collective Intelligence from a Synthetic and Biological Perspective Summer School (http://modelai.gettysburg.edu/2024/collective/) of which Prof Hönig (IMRC head) was one of the organizers. For more information about the differential drive controler
check out the /collision_avoidance/slides.pdf of the slides available on the website.


To start using my code, simply paste the files "J_controler.py" "J_state_estimator.py" "J_robot.py" "J_maths_module.py", as well as the trajectories folder and the logs folder in the root directory of your 3pi+ robot, alongside all software pre-installed by Pololu.
(NB in the repo the logs folder contains three examples of data log files and the corresponding pdf containing the plots. These files are not needed for the robot to work, you can delete them. The trajectories folder MUST contain the 3 pre-programmed trajectories though).
When you turn on your 3pi+ robot, simply select the "J_controler.py" program on the display screen and follow the instructions. (I use Kx = 1, Ky = 3, Ktheta = 3 for my gains).

The three pre-programmed trajectories are stored in a .json format. They contain a dictionary with lots of mostly useless info. The only two important items of the this dict are "states" and "actions" (if you want to modify or create your own trajectories, you can get rid of all the other items).
states : list of states the robots has to pass through during the trajectory. Each state is [x position, y position, angle theta]. A 0.1 sec interval is assumed between each state, meaning state 38 corresponds to 3.8 sec after the start of the trajectory.
actions : list of control actions that are needed by the controler

After a trajectory is executed, the data will automatically be saved in a .json file in the logs folder with an appropriate name. (NB sometimes the logfile doesn't show up immediately. Try restarting the robot). 

To plot the data, you can use the script "Plot_Pololu.py". The function plot_all() will automatically create the PDFs of all the logfiles in the logs folder, plotting them in regards to the correct ideal trajectory and naming the resulting PDF following the model of trajectoryname_Kx_Ky_Ktheta.pdf 
If a PDF with this name already exists (because for example there are multiple runs with the same trajectory and gains) it will a B at the end, as many times as needed. 
Just make sure to adjust the path given to plot_all() so that it points to the good directory.


Encountered issues during the project:

If the robot has very weird behavior (sudden acceleration in the wrong direction, not following the desired trajectory at all, moving erratically) one possible issue can be that the motor leads have been soldered the wrong way around. This causes positive speeds given to the motors to turn the motors in the negative direction
(meaning the controler wants the robot to move forward but it's actually driving backward). No panic though, the Pololu engineers thought about the issue : just open the file "Micropython/pololu_3pi_2040_robot/motors.py" and modify the attributes "self._flip_left_motor" and/or "self.flip_right_motor" until it works 
(ie until giving a positive speed to the motors makes the robot go in the direction where his bumpers are, not his USB port).

It happened two times that 3pi+ suddenly locked up its permissions and didn't allow me to modify, delete or add files (no write access). Using chmod command did not work as I got the response "read-only filesystem". The solution I found was updating the MicroPython firmware again (see 3pi+ 2040 user guide ; careful this will delete all custom files on the 3pi+) and then I could do chmod to get write access
