## FRC2429_2022

###Starting a new code repo for 2022 in robotpy.
Stripped-down version of the 2020 Python code for our robot
* includes a simulated drivetrain that takes advantage of all of the robotpy simulation capabilities (via the 2021 libraries).  
* This is a work in progress designed to integrate the robot simulation of autonomous and pathweaver trajectories with jupyter notebooks testing our code. 

Clone the git and install on your own machine:
Use "git clone https://github.com/aesatchien/FRC2429_2022.git" from the git bash (or any git aware) shell to download.  If you don't have git and you just want to look at the code, you can download the repository from the links on the right.

Notes on how to install python and the necessary accessories (particularly the robotpy libraries) that will get all of this running:
https://docs.google.com/document/d/1W679sj9CLOkiDFmzm6XB7qWhNopZQdBrFIUEkC-TWZM/edit?usp=sharing but may be a bit cryptic.  I'll help if you need it.

Once everything is installed, you need to go to the folder with robot.py.  From there, commands like

```python robot.py sim```

should bring up the simulator and allow you to check to see if your gamepad is recognized (you can also use the keyboard) and should be able to let you drive a virtual robot around the field if you have a gamepad. 

---
#### Organization:
* notebooks - folder with jupyter notebooks for analyzing telemetry from the robot (and the simulated robot)
* robot - folder with standard robotpy components: robot.py, physics.py etc
  * subsystems, commands, triggers - more classes for the robot
  * sim - folder containing images for the playing field and telemetry data exported from the robot
  * pathweaver - project folder for the trajectories (slalom, barrel, bounce) at 0.75 and 1.25 m/s- still optimizing these but slalom is quite good
  * guis - backups for the imgui.ini for the simulation interface
  
