#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt , pi,atan
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist,Point,Pose
import matplotlib.pyplot as plt

#Saturates PID outputs to actuator max capacity
def saturator(magnitude,value):
    if abs(value) < magnitude:
        return value
    elif value>magnitude:
        return magnitude
    else:
        return -magnitude

def convert(angle):
    #convert between [0,2pi]
    anglemod = angle % (2*pi)

    #convert between [-pi,pi]
    if anglemod > pi:
        anglemod -= 2*pi

    return anglemod


class Thymio : 
    def __init__(self): 

        rospy.init_node('robot_controller',anonymous=True)
         
        self.velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=10) 
        self.pose_subscriber = rospy.Subscriber('/odom',
                                                Odometry, self.update_pose)
        #sub= rospy.Subscriber("/thymio/scan", LaserScan, self.clbk_laser)
       
        self.pos=Point()
        self.pos.x=0
        self.pos.y=0 
        self.pos.z=0 
        self.theta=0
        self.rate = rospy.Rate(10)
        self.speed=[]
        self.positionx=[]
        self.positiony=[]

        print("x= {}, y = {}, theta ={}".format(self.pos.x, self.pos.y, self.theta))


    #callback from the position odometry topic 
    def update_pose(self,msg):
        self.pos.x=msg.pose.pose.position.x
        self.pos.y=msg.pose.pose.position.y
        self.pos.z=msg.pose.pose.position.z
        rot=msg.pose.pose.orientation 
        (_,_,self.theta)=euler_from_quaternion([rot.x,rot.y,rot.z,rot.w]) 

        
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pos.x), 2) +
                    pow((goal_pose.y - self.pos.y), 2))

    #P controller
    def linear_vel(self, goal_pose, kp=0.8):
        return saturator(0.23, kp * self.euclidean_distance(goal_pose))


    def absolute_angle(self, goal_pose):
        return convert(atan2((goal_pose.y - self.pos.y),(goal_pose.x - self.pos.x)))
        
    #P controller
    def angular_vel(self, goal_pose, ka=-10):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        #min(abs(self.steering_angle(goal_pose) - self.convert(self.theta)),abs(2*pi-self.steering_angle(goal_pose) - self.convert(self.theta)))
        return saturator(0.7,ka * convert(convert(self.absolute_angle(goal_pose)) - convert(self.theta)))


    def move2goal(self, goal_pose):
        """Moves the thymio to the goal."""

        distance_tolerance = 0.05 #input("Set your tolerance: ")
        angular_tol = 0.02

        vel_msg = Twist() 

        #change it to while goal pose non null
        while(len(goal_pose)>0):

            

            #The process only stops once the goal is reached

            while self.euclidean_distance(goal_pose[0]) > distance_tolerance:

                #init so it enters the while first time
                vel_msg.angular.z= 10*angular_tol
                #priority is given to the rotation, so robot orients itself when it is needed
                while abs(vel_msg.angular.z) > angular_tol :

                    vel_msg.angular.z =self.angular_vel(goal_pose[0])
                    vel_msg.linear.x  = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    # Publishing our vel_msg
                    print("vx= {},theta ={}".format(vel_msg.linear.x, vel_msg.angular.z))
                    print("Distance remaining: {}".format(self.euclidean_distance(goal_pose[0])))
                    
                    self.velocity_publisher.publish(vel_msg)

                    # Publish at the desired rate.
                    self.rate.sleep()

                #while distance decreasing at nice pace?
                for i in range(30):

                    vel_msg.angular.z = 0
                    vel_msg.linear.x = self.linear_vel(goal_pose[0])
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    # Publishing our vel_msg
                    print("vx= {},theta ={}".format(vel_msg.linear.x, vel_msg.angular.z))
                    print("Distance remaining: {}".format(self.euclidean_distance(goal_pose[0])))
                    self.velocity_publisher.publish(vel_msg)
                    self.speed.append(vel_msg.linear.x) 
                    self.positionx.append(self.pos.x)
                    self.positiony.append(self.pos.y)
                    # Publish at the desired rate.
                    self.rate.sleep()

    


            # Stopping our robot after the movement is over.
            print("Intermediate goal reached")
            goal_pose.pop(0)

        print("End of path")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin() 

if __name__ == '__main__':

    ## Hacky test initialization

    goal_pose = []

    for i in range(3):
        nextPoint = Point() 
        # input from the user.
        nextPoint.x = input("Set your x goal: " )
        nextPoint.y = input("Set your y goal: " )
        goal_pose.append(nextPoint)

    ## end of init 

    try:
        rob = Thymio()
        rob.move2goal(goal_pose)
        plt.figure(1)
        plt.plot(rob.speed)
        plt.show()
        plt.savefig("speed.png")
        plt.figure(2) 
        plt.plot(rob.positionx)
        plt.plot(rob.positiony)
        plt.show()
        plot.savefig("position.png")
    except rospy.ROSInterruptException:
        pass