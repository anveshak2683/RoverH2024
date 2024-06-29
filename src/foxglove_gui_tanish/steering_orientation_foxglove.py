import rospy
import sys
from std_msgs.msg import Float32MultiArray, Float32


class steering_orientation_foxglove:
    def __init__(self):
        self.enc_auto_sub = rospy.Subscriber(
            "/enc_auto", Float32MultiArray, self.enc_auto_callback)
        self.enc_data = [0,0,0,0]
        self.left_front_pub = rospy.Publisher(
            "left_front", Float32, queue_size=10)
        self.right_front_pub = rospy.Publisher(
            "right_front", Float32, queue_size=10)
        self.right_back_pub = rospy.Publisher(
            "right_back", Float32, queue_size=10)
        self.left_back_pub = rospy.Publisher(
            "left_back", Float32, queue_size=10)

    def enc_auto_callback(self, msg):
        self.enc_data[0] = msg.data[0]  # left-front
        self.enc_data[1] = msg.data[3]  # right-front
        self.enc_data[2] = msg.data[2]  # left-back
        self.enc_data[3] = msg.data[5]  # right-back

    def main(self):
        angles = [0, 0, 0, 0]
        for i in range(0, 4):
            if (self.enc_data[i] % 180) > 90:
                angles[i] = -(180-(self.enc_data[i] % 180))
            else:
                angles[i] = self.enc_data[i] % 180
        self.left_front_pub.publish(angles[0])
        self.right_front_pub.publish(angles[1])
        self.left_back_pub.publish(angles[2])
        self.right_back_pub.publish(angles[3])
        print("publishing angles lf,rf,lb,rb:", angles)

    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            # rate = rospy.Rate(10)
            # rate.spin()


def main(args):
    sef = steering_orientation_foxglove()
    rospy.init_node('steering_orientation_foxglove', anonymous=True)

    try:
        sef.spin()
        # rate = rospy.Rate(10)
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == '__main__':
    main(sys.argv)
