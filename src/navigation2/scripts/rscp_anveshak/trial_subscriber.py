import rospy
from std_msgs.msg import Int8

class Receiver:
    def __init__(self):
        rospy.init_node('sub_rscp')
        rospy.Subscriber("/msg_id", Int8, self.id_callback)
         
    def id_callback(self, msg):
        print("Data received ", msg)

    def main(self):
        pass

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

if __name__ == '__main__':
    receiver = Receiver()
    receiver.spin()
