import math
import time
import thread
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion
from helpers.quaternion_to_euler import euler_from_quaternion as quat_to_eul
from helpers.euler_to_quaternians import get_quaternion_from_euler as eul_to_quat
import numpy as np


class MovmentController():
    def __init__(self):
        self.movement_velocity = 0.0
        # let's turn at 0 radians/s
        self.angular_velocity = 0
        self.global_pose=None
    def set_global_pose(self,pose):
        self.global_pose=pose
    def get_global_pose(self):
        return self.global_pose
    def set_speeds(self,movement_v,angular_v):
        self.set_movement_velocity(movement_v)
        self.set_angular_velocity(angular_v)
    def set_movement_velocity(self,movement_velocity):
        use_custom_speeds=True
        self.movement_velocity=movement_velocity#in m/s
    def set_angular_velocity(self,angular_velocity):
        use_custom_speeds=True
        self.angular_velocity=angular_velocity #in radians/s
    def apply_speed(self):
        while not rospy.is_shutdown():
            if use_custom_speeds:
                move_cmd = Twist()
                move_cmd.linear.x = self.movement_velocity
                move_cmd.angular.z = self.angular_velocity
                #print("applying current speed:", move_cmd)
                cmd_vel.publish(move_cmd)
                #print(self.movement_velocity,self.angular_velocity)
            time.sleep(0.3)

class Bouncer():
    '''Class for bouncing robot around.'''

    def __init__(self):
        self.should_patroll=False
        self.min_distance = 1.3
        self.target_min_distance = 0.8
        self.too_close = False
        self.range_min = 0

        self.fov = 100#88

        self.tolerance = 5

        self.BURGER_MAX_LIN_VEL = 0.20  # 0.22
        self.BURGER_MAX_ANG_VEL = 0.92  # 2.84

        self.WAFFLE_MAX_LIN_VEL = 0.26
        self.WAFFLE_MAX_ANG_VEL = 1.82

        self.LIN_VEL_STEP_SIZE = 0.01
        self.ANG_VEL_STEP_SIZE = 0.1

        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        self.target_angle = 0

        self.lidar_ranges = []
        self.heading = 0

        self.turtlebot3_model = "waffle"

        self.sub1 = rospy.Subscriber('/scan', LaserScan, self.tooClose)
        self.sub2 = rospy.Subscriber('/odom', Odometry, self.getOrientation)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        rospy.on_shutdown(self.stopOnShutdown)

        self.twist = Twist()

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input

        return output

    def getOrientation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        self.heading = int(euler_from_quaternion(orientation_q_list)[2] * (180 / np.pi) + 180)

    def tooClose(self, msg):
        self.range_min = msg.range_min
        self.lidar_ranges = np.array(msg.ranges)
        view = np.array(self.lidar_ranges[0:self.fov / 2] + self.lidar_ranges[(359 - self.fov / 2):359])
        closest_front = np.min(view[self.range_min < view])
        if (closest_front < self.min_distance):
            print("Too Close!")
            movement_controller.set_speeds(0,0)
            self.too_close = True

    def stopOnShutdown(self):
        ''' Stop robot when shutting down '''

        rospy.loginfo("System is shutting down. Stopping robot...")
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        #self.pub.publish(self.twist)

    def isTargetSafe(self):
        low = self.target_angle - (self.fov / 2)
        high = self.target_angle + (self.fov / 2)

        view = np.arange(low, high)

        if (low < 0):
            low = 360 + low
            view = np.concatenate((np.arange(low, 360), np.arange(0, self.target_angle)))
        elif (high > 360):
            high = high - 360
            view = np.concatenate((np.arange(0, high), np.arange(self.target_angle, 360)))

        view_bool = (self.range_min < self.lidar_ranges[view]).all() and (
                    self.lidar_ranges[view] < self.target_min_distance).all()
        return view_bool

    def randomTurn(self):
        while True:
            self.target_angle = (self.heading+np.random.randint(90, 270))%359
            print(self.target_angle)
            if (self.isTargetSafe()):
                self.target_angle = (self.heading+np.random.randint(90, 270))%359
            else:
                break

        r = rospy.Rate(100)
        while True:
            # print("Heading: %d" % self.heading)
            # print("Target: %d" % self.target_angle)
            if (self.target_angle - self.tolerance <= self.heading <= self.target_angle + self.tolerance):
                self.twist.angular.x = 0.0
                self.twist.angular.y = 0.0
                self.twist.angular.z = 0.0
                movement_controller.set_speeds(0, 0)
                #self.pub.publish(self.twist)
                self.too_close = False
                return

            self.target_angular_vel = self.WAFFLE_MAX_ANG_VEL
            self.target_linear_vel = 0.0
    ##
            self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel,
                                                             (self.LIN_VEL_STEP_SIZE / 2.0))
            #self.twist.linear.x = self.control_linear_vel;
            #self.twist.linear.y = 0.0;
            #self.twist.linear.z = 0.0
#
            self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel,
                                                               (self.ANG_VEL_STEP_SIZE / 2.0))
            #self.twist.angular.x = 0.0;
            #self.twist.angular.y = 0.0;
            #self.twist.angular.z = self.control_angular_vel
            #movement_controller.set_speeds(0, self.control_angular_vel/2)
            movement_controller.set_speeds(0, 0.9)
            # self.pub.publish(self.twist)
            r.sleep()

    def goStraight(self):
        print("Moving...")
        #self.target_angular_vel = 0.0
        #self.target_linear_vel = self.BURGER_MAX_LIN_VEL
#
        #self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel,
        #                                                 (self.LIN_VEL_STEP_SIZE / 2.0))
        #self.twist.linear.x = self.control_linear_vel;
        #self.twist.linear.y = 0.0;
        #self.twist.linear.z = 0.0
#
        #self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel,
        #                                                  (self.ANG_VEL_STEP_SIZE / 2.0))
        #self.twist.angular.x = 0.0;
        #self.twist.angular.y = 0.0;
        #self.twist.angular.z = 0.0
        movement_controller.set_speeds(0.4, 0)
        #self.pub.publish(self.twist)


def read_pose_from_robot(msg):
    #is used as callback, therefore position is updated in realtime
    #print("setting pose:",msg.pose)
    movement_controller.set_global_pose(msg.pose)
def get_global_pose():
    #takes the saved pose and parses it to a map
    pose=movement_controller.get_global_pose()
    print("reading_pose:", pose)
    result={
        'x':pose.pose.position.x,
        'y':pose.pose.position.y,
        'z':pose.pose.position.z,
        'quant_x':pose.pose.orientation.x,
        'quant_y': pose.pose.orientation.y,
        'quant_z': pose.pose.orientation.z,
        'quant_w': pose.pose.orientation.w,
    }
    return result
def patroll():
    #print("is patrolling:", bounce.should_patroll)
    #print(movement_controller.movement_velocity,movement_controller.angular_velocity)
    if bounce.should_patroll:
        if (bounce.too_close):
            bounce.randomTurn()
        bounce.goStraight()

class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):
        print("called goto")
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)
        print("move started")

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))
        print("end of movement task")
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def interupt_patroll():
    bounce.should_patroll=False
    movement_controller.set_speeds(0,0)
def resume_patroll():
    bounce.should_patroll=True

def navigate_to_OTM(robot_global_position,OTM_relative_position,offset=0.13):
    """
    Function that will move the turtlebot towards the OTM. After the Movement the robot will be
    offset meters away from the OTM (and have an angle of 0 degrees)
    :param robot_global_position: Map including x,y,z,quant_x,quant_y,quant_z,quant_w
    :param OTM_relative_position: Map with dist and angle
    :param offset: distance how far infront of the OTM the robot will stop
    :return:
    """
    #moves the robot towards the OTM and returns the new relative distance of OTM and robot centre
    rp=robot_global_position
    ot=OTM_relative_position
    ###Determine camera position###
    _,_,beta= quat_to_eul(rp["quant_x"],rp['quant_y'],rp['quant_z'],rp['quant_w'],indeg=True)
    print(beta)
    position_camera={'x':rp["x"]+math.cos(math.radians(beta))*0.08,
                    'y': rp["y"]+math.sin(math.radians(beta))*0.08}
    ###Detemine OTM position###
    angle_to_otm=beta-ot["angle"]
    print("angle_to_OTM:",angle_to_otm)
    position_otm = {'x':position_camera["x"]+
                        math.cos(math.radians(angle_to_otm))*ot["dist"]
                    ,
                    'y': position_camera["y"]+
                         math.sin(math.radians(angle_to_otm))*ot["dist"]
                    }
    print("position_robot:",rp)
    print("position_camera:",position_camera)
    print("pos_otm:",position_otm)
    ###Make sure robot stops slightly before OTM position###
    position_desired={'x':position_otm["x"]-math.cos(math.radians(angle_to_otm))*offset,
                      'y':position_otm["y"]-math.sin(math.radians(angle_to_otm))*offset}
    r1,r2,r3,r4=eul_to_quat(0,0,math.radians(angle_to_otm))
    quaternion_desired = {'r1': r1, 'r2': r2, 'r3': r3, 'r4': r4}

    delta_x=position_desired["x"]-position_otm["x"]
    delta_y=position_desired["y"]-position_otm["y"]
    distance_otm_stop=math.sqrt(delta_x*delta_x+delta_y*delta_y) #measured from robot centre, not from camera
    print("moving_to:",position_desired,quaternion_desired)
    print("distance_otm_stop:",distance_otm_stop, " should be: ",offset)

    goToPose.goto(pos=position_desired,quat=quaternion_desired)
def move_to_dump():
    """
    Moves the Turtlebot to the dump position in preparation of OTM unloading
    """
    position_desired = dump_position #defined in setup
    r1, r2, r3, r4 = eul_to_quat(0, 0, math.radians(0))
    quaternion_desired = {'r1': r1, 'r2': r2, 'r3': r3, 'r4': r4}
    goToPose.goto(position_desired,quaternion_desired)




def movement_setup():
    print("\nSetting up movement manager")
    #rospy.init_node('Movement_manager', anonymous=False)
    global cmd_vel
    global movement_controller
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    movement_controller = MovmentController()
    movement_controller.set_speeds(0.0, 0.0)
    thread.start_new_thread(movement_controller.apply_speed, ())
    r = rospy.Rate(10);
    ###Setup Read_position####
    odom_sub = rospy.Subscriber('/odom', Odometry, read_pose_from_robot)
    #global bounce
    #bounce=Bouncer()
    global use_custom_speeds
    use_custom_speeds=True
    global goToPose
    goToPose=GoToPose()

    global dump_position
    dump_position={'x':0.0,'y':0.0}
    print ("Movement_controller setup is complete\n")


if __name__ == '__main__':
    movement_setup()

