## FRC2429_2022

### Code repo for 2022 using robotpy
FRC robot code for the 2022 robot using robotpy-commands-v2.  Latest update is after AVR regionals.  **Code needs end of year cleaning.**
* includes a simulated drivetrain that takes advantage of all of the robotpy simulation capabilities (still uses some of the 2021 libraries).  
* This is a work in progress designed to integrate the robot simulation of autonomous and pathweaver trajectories with jupyter notebooks testing our code. 

---
#### Organization
* robot - folder with standard robotpy components: robot.py, physics.py etc for the 2022 bot
  * subsystems, commands, triggers - more classes for the robot
  * sim - folder containing images for the playing field and telemetry data exported from the robot
  * pathweaver - project folder for the trajectories (needed for the 2, 3 and 4-ball autonomous)
 * gui - pyqt project that gives us our own python-based driver station dashboard (works with sim and robot)
 * templates - robot code for students to practice w/o fear of breaking the main robot (mainly from the robotpy examples)
   * template_auto - a place to try autonomous homework and sim it
   * template_charbot - just the drivetrain and an autonomous that resoponds to frc-characterization tool so we can get good inputs to trajectories
   * template_hatch - a pre-built simple bot with a pneumatics subsystem
   * template_ramsete - pre-built robot that can follow a ramsete (trajectory) command
   * template_romi - pre-built that lets you run the romi remotely just using the sim 
     * no driver station needed! just use your laptop and connect to the romi's 2429_RedRomi network
     * run `python robot.py sim --ws-client` to use it
     * 
#### Where's the other stuff?
* training notebooks are here: https://github.com/aesatchien/FRC_training
* vision system (image processing on the pi) is here:  https://github.com/aesatchien/FRC2429_vision
---
#### Installation
Clone the git and install on your own machine:
Use "git clone https://github.com/aesatchien/FRC2429_2022.git" from the git bash (or any git aware) shell to download.  If you don't have git and you just want to look at the code, you can download the repository from the links on the right.

Notes on how to install python and the necessary accessories (particularly the robotpy libraries) that will get all of this running:
https://docs.google.com/document/d/1oS4aMhn9Rf_kpubbQ_JGEtOcRR36LoU-QbJQERZtyIU/edit#heading=h.665ussze99ev but may be a bit cryptic.  I'll help if you need it.
basically, in your python environment you'll need a `pip install robotpy[all]` and if you are working on anything else several more packages.

Once everything is installed, you need to go to the folder with robot.py.  From there, commands like

```python robot.py sim```

should bring up the simulator and allow you to check to see if your gamepad is recognized (you can also use the keyboard) and should be able to let you drive a virtual robot around the field if you have a gamepad. 
