import rospy
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class Cupping_Encoder():
    def __init__(self):
        rospy.Subscriber("/enc_auto", Float32MultiArray, self.enc_clbk)
        self.enc_pub = rospy.Publisher("/cupping_enc", Odometry, queue_size=10)
        self.what_should_be_changed = 0.0

    def enc_clbk(self, msg):
        num = 0
        self.what_should_be_changed = msg.data[num]
        enc = Odometry()
        x,y,z,q = quaternion_from_euler(0,0,self.what_should_be_changed)
        enc.p
        self.enc_pub.publish(enc)
        print("enc", enc)
        print("callback hit")

    def main(self):
        print("what should be changed:", self.what_should_be_changed)

    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

if __name__ =="__main__":
    rospy.init_node("cupping_encoder", anonymous=True)
    rate = rospy.Rate(10)
    auto = Cupping_Encoder()
    auto.spin()
	
	
