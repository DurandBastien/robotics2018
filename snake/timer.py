import rospy
import time
from std_msgs.msg import String



def talker():
    tailExtensionTime = 30
    pub = rospy.Publisher('time', String, queue_size=1)
    rospy.init_node('timer', anonymous=True)
    while not rospy.is_shutdown():
        pub.publish("Time!")
        time.sleep(tailExtensionTime)

if __name__ == '__main__':
    try:
        message()
    except rospy.ROSInterruptException:
        pass
