# ARGroupProject2024

Run the following scripts in separated terminals.  
  
    a) Run the roscore to initiate  
        `roscore`  

    b) Run speech_recognition module  
        `rosrun rchomeedu_speech google_sr.py`  

    c) Run sound play module  
        `rosrun sound_play soundplay_node.py`  
        `rosrun sound_play say.py "Welcome to Pose Detection Robot!"` 

    d) Initiate the detector script  
        `cd catkin_ws/src/ROS_fall_detection/src`  
        `python detector.py`  

    e) Initiate the camera script
        `cd catkin_ws/src/ROS_fall_detection/src`  
        `python cam.py` 

    f) Run the output program  
        `cd catkin_ws/src/`  
        `python pose_detector.py`  

Congratulations, the system is up running!
