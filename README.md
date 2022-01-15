# chronus
This repository contains media art project components I worked on from June to December 2021.  
Made on Linux distro [Ubuntu](https://ubuntu.com/) and [Firefox](https://www.mozilla.org/en-US/firefox/new/).

## About the Projects
### Unfamiliar Convenient  
<img src="https://i.imgur.com/R0CSiM5.jpg" width="50%" />  <br/>
[Unfamiliar Convenient](https://vjnks.com/works/unfamiliar-convenient-46) is set to challenge the limits of smart home objects in order to introduce a range of peculiar behaviors. The first inquiry in an envisioned series studies the relationship between a voice assistant and a vacuum cleaner. The project aims to draw a clearer distinction between the two often coupled definitions, the "internet of things" and "the smart home", as a prerequisite to imagine alternatives for dominant servitude-driven relationships with everyday technologies. What if domestic appliances were to be considered a species? 

### Altar-3000
<img src="https://i.imgur.com/pQiZ6BA.jpg" width="50%" /> <br/>
[ALTAR-3000](https://vjnks.com/works/altar-3000-51) is a long overdue, automated totem of customised belief. Powered by natural language processing, this intelligent home altar performs gimmicky New Age rituals all while generating "prophecies" for headlines retrieved from betting platforms that offer exchanges on near-future political, financial, and other events. The automated sand garden—adorned with paraphernalia that might have meant something once but doesn’t reflect much in the age of multitude of easily digestible truths—disseminates its prophecies on conspiracy forums.

## Built With
### Unfamiliar Convenient  
Refer to the ```roomba``` and ```settings-gui``` directory and its respective ```README.md``` for **Getting Started**.
#### ```roomba```
Collecting satellite data to start the roomba's ritual
* [Arduino](https://www.arduino.cc/)

#### ```settings-gui```
A simple settings gui for exhibition technicians to edit fields from ```oikomancy.py```, ML file, which will adjust and match roomba's action to the physical environment.
* [Flask](https://flask.palletsprojects.com/en/2.0.x/)
* HTML/CSS
* Python 

### ALTAR-3000
Refer to the ```altar-3000-py-txt-files``` directory for final implementation
#### ```arm_commands``` directory 
Utilizes ```arm-movement/dofbot_kinematics.cpp``` to produce ```txt``` files for servo degrees with ```ROS rvis```.  
Models found in [DOFBOT-Jetson repository](http://www.yahboom.net/study/Dofbot-Jetson_nano).
* [ROS](https://www.ros.org/)
* [MoveIt! Framework](https://moveit.ros.org/)
* C++

#### ```rake-action.py ``` and ```circle-action.py```  
* Python 
* [Arm_lib](https://pypi.org/project/Arm_lib/)

#### ```selenium_driver.py``` 
Firefox web controller to send prophecies to [8kun](https://en.wikipedia.org/wiki/8chan) and to extract data for the [Open Frameworks](https://openframeworks.cc/) visual interaction. 
* Python

## Acknowledgements
* [Understanding DOF and Kinematics](https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps)
* [QAnon and Conspiracy](https://www.nytimes.com/article/what-is-qanon.html)
* [Yahboom DOFBOT-Jetson repository](http://www.yahboom.net/study/Dofbot-Jetson_nano)
* [Chronus Art Center](http://www.chronusartcenter.org/en/)