'''
### Steps to Install ROS Melodic in case errors
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install ros-melodic-sound-play ros-melodic-audio-common

### Step to Install Twilio - This is to make actual calls
sudo apt-get install python-pip
pip install twilio==6.0.0  # Version 6.0.0 is compatible with Python 2.7

### Creating alert_notification package
cd ~/catkin_ws/src
catkin_create_pkg alert_notification std_msgs rospy roscpp
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
'''

# MAIN CODE HERE
# alert_notification/src/alert_node.py

#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import message_filters
import subprocess
import time
# from sound_play.libsoundplay import SoundClient
# from twilio.rest import Client [Uncheck if using Twilio - necessary for making calls]

# Uncheck the following section if using Twilio
'''
def make_call():
    # Twilio credentials - May use config files to store these
    account_sid = 'ACeb1c0a90633b6766ac90cf9014995cde' # Input the ID here
    auth_token = '495aaf19bdbdea37d9a3013c33bc3835' # Input the Authentication Token here
   
    # Create a Twilio client instance with the provided credentials
    client = Client(account_sid, auth_token)

    # Make an outgoing call using Twilio's API
    call = client.calls.create(
        twiml='<Response><Say>Alert! Fall detected! Calling for help.</Say></Response>',
        to='+60182767347', # Replace with the emergency contact
        from_='+14788873902' # Replace with Twilio phone number
    )

    # Log the unique SID of the initiated call for reference
    rospy.loginfo("Call SID: %s", call.sid)
'''

class Server:
    def __init__(self):
        self.keyword = ""
        self.detection = ""
        self.global_lock = False #workaround to 

    def keyword_callback(self,msg):
        self.keyword = msg.data
        rospy.loginfo(self.keyword)
       

    def detection_callback(self,msg):
        self.detection = msg.data
        self.compute()

    def compute(self):
        if self.global_lock == True: return
        else: self.global_lock=True
        # Keyword represents the user responds
        # Detection represents the fall detection system response
        if self.detection == "Yes":
            subprocess.check_output(['rosrun', 'sound_play', 'say.py', "Do you need help? Do you need help?"])
            time.sleep(5)
            if "yes" in self.keyword.lower() and self.detection == "Yes":
                rospy.loginfo("Playing alert sound: 'Alert! Fall detected! Calling for help.'")
                subprocess.check_output(['rosrun', 'sound_play', 'say.py', "Alert! Fall detected! Calling for help."])
                # Simulate calling for help by logging to the console
                rospy.loginfo("Simulating a call to emergency contact.")
            elif "no" in self.keyword.lower() and self.detection == "Yes":
                rospy.loginfo("Fall detected but No help needed.'")
                subprocess.check_output(['rosrun', 'sound_play', 'say.py', "No help needed. No help needed."])
               
        elif self.detection == "No":
            rospy.loginfo("Normal condition.")
        time.sleep(10)# stop function keep getting pinging
        self.global_lock=False




# Main function that initializes the node and subscribes to the 'fall_alert' topic
def alert_node():
    rospy.init_node('alert_node', anonymous=True)

    server = Server()
    server.compute()

    # Recieve responses from google_sr.py
    sub1 = rospy.Subscriber('/result', String, server.keyword_callback)

    # Recieve responses from detector.py
    sub2 = rospy.Subscriber('/pub_result', String, server.detection_callback)

    rospy.loginfo("Alert Notification Node has started and is waiting for alerts...")
    rospy.spin()

# Entry point of the script
if __name__ == '__main__':
    try:
        alert_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Alert Notification Node has been terminated.")
