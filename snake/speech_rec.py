#!/usr/bin/env python
import speech_recognition as sperec
import rospy
from std_msgs.msg import String

    
def wordlist():
    return ["begin", "stop"]

def micListener():
    pub = rospy.Publisher('voice_commands', String, queue_size=10)
    rospy.init_node('voice_recogniser', anonymous=True)
    rospy.loginfo("Node created")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        result = listen()
        accepted = False
        if (result == "REQUERROR"):
            rospy.loginfo("Connection error.  Speech recognition will not work until connection reestablished.")
        else:
            if(result != "NOVD"):
                rospy.loginfo(result) #Very much for debugging purposes.  Comment out if you don't want the code to report everything it hears.
                for word in wordlist():
                    if (result.find(word) != -1) and (not accepted):
                        rospy.loginfo(word)
                        pub.publish(word)
                        accepted = True

def listen():
    rec = sperec.Recognizer()
    rospy.loginfo("rec setup")
    mic = sperec.Microphone()
    rospy.loginfo("mic setup")
    with mic as audiosource:
        rec.adjust_for_ambient_noise(audiosource, duration=0.7)
        rospy.loginfo("ambient noise adjusted")
        audio = rec.listen(audiosource)
        rospy.loginfo("listened")
    try:
        input = rec.recognize_google(audio)
        return input
    except sperec.RequestError:
        return "REQUERROR"
    except sperec.UnknownValueError:
        return "NOVD"

if __name__ == '__main__':
    try:
        micListener()
    except rospy.ROSInterruptException:
        pass
