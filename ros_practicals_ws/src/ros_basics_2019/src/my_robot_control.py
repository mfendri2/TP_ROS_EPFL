#!/usr/bin/env python
#!/usr/bin/env python
import pickle
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt , pi,atan
from tf.transformations import euler_from_quaternion 
from geometry_msgs.msg import Twist,Point,Pose

goal_pose = []

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
        self.command_subscriber = rospy.Subscriber('/goal_pos',
                                                Point, self.receive_command)
       
        self.pos=Point()
        self.pos.x=0
        self.pos.y=0 
        self.pos.z=0 
        self.theta=0
        self.rate = rospy.Rate(10)
        self.speed=[] 
        self.positionx=[] 
        self.positiony=[]
        print("Initial Position: x= {}, y = {}, theta ={}".format(self.pos.x, self.pos.y, self.theta))

    def receive_command(self,msg):
        print('received direction to ({},{})'.format(msg.x,msg.y))
        goal_pose.append(msg)



    #callback from the position odometry topic 
    def update_pose(self,msg):
        self.pos.x=msg.pose.pose.position.x
        self.pos.y=msg.pose.pose.position.y
        self.pos.z=msg.pose.pose.position.z
        rot=msg.pose.pose.orientation 
        (_,_,self.theta)=euler_from_quaternion([rot.x,rot.y,rot.z,rot.w]) 

        
    def euclidean_distance(self, goal):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal.x - self.pos.x), 2) +
                    pow((goal.y - self.pos.y), 2))

    #P controller
    def linear_vel(self, goal, kp=0.8):
        return saturator(0.23, kp * self.euclidean_distance(goal))


    def absolute_angle(self, goal):
        return convert(atan2((goal.y - self.pos.y),(goal.x - self.pos.x)))
        
    #P controller
    def angular_vel(self, goal, ka=-10):
        #min(abs(self.steering_angle(goal_pose) - self.convert(self.theta)),abs(2*pi-self.steering_angle(goal_pose) - self.convert(self.theta)))
        return saturator(0.7,ka * convert(convert(self.absolute_angle(goal)) - convert(self.theta)))

   
    def go(self):
        while(True):
            if(len(goal_pose)>0):
                print('To infinity, and beyond! ')
                rob.move2goal()
        # Figure out where to place this: rospy.spin() maybe create a function with all the above block anmd put spin on last line without the while loop
        #rospy.spin()


    def move2goal(self):
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
                    #print("Turning: theta ={}".format( vel_msg.angular.z))
                    self.speed.append(vel_msg.linear.x) 
                    self.positionx.append(self.pos.x)
                    self.positiony.append(self.pos.y)
                    self.velocity_publisher.publish(vel_msg)
                    with open('speed', 'wb') as f: 
                        pickle.dump(self.speed, f)
                    with open('postionx','wb') as f:
                        pickle.dump(self.positionx,f)
                    with open('postiony','wb') as f:
                        pickle.dump(self.positiony,f)

                    # Publish at the desired rate.
                    self.rate.sleep()

                #while distance decreasing at nice pace?
                for i in range(30):

                    vel_msg.angular.z = 0
                    vel_msg.linear.x = self.linear_vel(goal_pose[0])
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    # Publishing our vel_msg
                    #print("Moving: vx= {}".format(vel_msg.linear.x))
                    #print("Distance remaining: {}".format(self.euclidean_distance(goal_pose[0])))
                    self.speed.append(vel_msg.linear.x) 
                    self.positionx.append(self.pos.x)
                    self.positiony.append(self.pos.y)
                    self.velocity_publisher.publish(vel_msg)
                    with open('speed', 'wb') as f: 
                        pickle.dump(self.speed, f)
                    with open('postionx','wb') as f:
                        pickle.dump(self.positionx,f)
                    with open('postiony','wb') as f:
                        pickle.dump(self.positiony,f)

                    # Publish at the desired rate.
                    self.rate.sleep()
                #rospy.spin() 


    


            # Stopping our robot after the movement is over.
            print("Intermediate goal reached")
            goal_pose.pop(0)
            print('{} objectives remaining'.format(len(goal_pose)))

        print("End of path")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin() 

if __name__ == '__main__':

    del goal_pose[:]

    ## Hacky test initialization
    for i in range(0):
        nextPoint = Point() 
        # input from the user.
        nextPoint.x = input("Set your x goal: " )
        nextPoint.y = input("Set your y goal: " )
        goal_pose.append(nextPoint)
    ## end of init 

    try:
        rob = Thymio()
        rob.go()
        plt.figure()
        plt.plot(rob.speed)
        plt.xlabel("Time sample")
        plt.ylabel("linear speed [m/s]")
        plt.grid(True)
        plt.savefig("Speed.png")

        plt.figure()
        plt.plot(rob.positionx)
        plt.plot(rob.positiony)
        plt.xlabel("Time sample")
        plt.ylabel("position [m]")     
        plt.legend(("Postion in x axis","postion in y axis "))
        plt.savefig("postion.png")   
    except rospy.ROSInterruptException:
        pass