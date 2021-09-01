from controller import Robot, LidarPoint
import math
from math import pi
import time

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Obtain waypoints
waypoints = []
waypoints_string = robot.getCustomData().split()
for i in range(10):
    waypoints_element = [float(waypoints_string[2*i]), float(waypoints_string[2*i+1])]
    waypoints.append(waypoints_element)
print('Waypoints:', waypoints)
# Initialize devices
motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')
l1 = robot.getDevice('Hokuyo URG-04LX-UG01')


motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)
gps.enable(timestep)
imu.enable(timestep)
l1.enable(timestep)
l1.enablePointCloud()


######################################################################
##Variables

x_obs,y_obs=0,0
fie=0
fie_rad=0
diff_x,diff_y=0,0


A=1
obs_x=[0]*667
obs_y=[0]*667
final_x_obs=0
final_y_obs=0
new_waypoint=[]
shortest=0
closest_waypoint=[0,0]
B=0
################################################################


def rayToDistance(ray):
        px = point[ray].x
        py = point[ray].z

        dist = math.dist([px,py],[0,0])
        return dist


n=0
v=0


xi=gps.getValues()[0]
yi=gps.getValues()[1]
timeCount=0
diff=0
time=180 
diffAngle=0
OA=0
WP=1
frontArea = []



OBSTACLE_DIST = 0.6

SafeLine = 1
REGIONAL_ANGLE = 28
PI = 3.141592653


NORMAL_LIN_VEL = 0.50 
TRANS_LIN_VEL = -0.08
TRANS_ANG_VEL = 1.5

REGIONS = [
                "front_C", "front_L", "left_R",
                "left_C", "right_C", "right_L", "front_R",
            ]

Urgency_Report = {
                    "goBackCount": 0, "act": False, "angular_vel": 0.0, "sleep": 0
                    }

Regions_Report = {
                        "front_C":[], "front_L":[], "left_R":[],
                        "left_C":[], "right_C":[], "right_L":[], "front_R":[],
                    }

Regions_Distances = {
                        "front_C": 0, "front_L": 1, "left_R": 2,
                        "left_C": 3, "right_C": -3, "right_L": -2, "front_R": -1,
                    }


def ClearanceTest():
    global Urgency_Report
    count = 0
    goal = "front_C"
    closest = 10e6
    regional_dist = 0
    maxima = {"destination": "back_C", "distance": 10e-6}
    for region in Regions_Report.items():
        regional_dist = abs(Regions_Distances[region[0]]-Regions_Distances[goal])
        #if there're no obstacles in that region
        if not len(region[1]):
            count += 1
            #check if it's the cheapest option
            if (regional_dist < closest):
                closest = regional_dist
                maxima["distance"] = OBSTACLE_DIST
                maxima["destination"] = region[0]
        #check if it's the clearest option
        elif(max(region[1]) > maxima["distance"]):
            maxima["distance"] = max(region[1])
            maxima["destination"] = region[0]
    #calculate the cost to the chosen orientation
    # print(count)
    regional_dist = Regions_Distances[maxima["destination"]]-Regions_Distances[goal]

    #we act whenever the clearest path is not the front_C (front center)
    Urgency_Report["goBackCount"] = (count)
    Urgency_Report["act"] = (closest != 0)
    Urgency_Report["angular_vel"] = ((regional_dist/max(1, abs(regional_dist)))
                                    *TRANS_ANG_VEL)
    Urgency_Report["sleep"] = ((abs(regional_dist)*REGIONAL_ANGLE*PI)
                                /(180*TRANS_ANG_VEL))


def linearpid():
    global targetCoordinate
    print("enter linear pid")
    d = math.dist(targetCoordinate , [gps.getValues()[0],gps.getValues()[1]])
    vel=1.8*d      
    motor_left.setVelocity(vel)
    motor_right.setVelocity(vel)
    print("d is :",d)
    if(d<0.2):
        print("enter critical linear pid")
        # A=1
        # C=0
        motor_left.setVelocity(0.0)
        motor_right.setVelocity(0.0)


def IdentifyRegions():
    global Regions_Report
    for i, region in enumerate(REGIONS):
        if(i<=3):
            Regions_Report[region] = [
                    rayToDistance(x) for x in range(372-78*i,372-78*(i+1),-1)
                            if rayToDistance(x) <= OBSTACLE_DIST and rayToDistance(x) != 'inf']
        elif(i>3):
            Regions_Report[region] = [
                    rayToDistance(x) for x in range(606-78*(i-4),606-78*(i-3),-1)
                            if rayToDistance(x) <= OBSTACLE_DIST and rayToDistance(x) != 'inf']  
    
def Steer():
    global Urgency_Report

    #since we're moving only on the plane, all we need is move in the x axis,
    #and rotate in the z (zeta) axis.
    motor_left.setVelocity(-Urgency_Report["angular_vel"] + TRANS_LIN_VEL)
    motor_right.setVelocity(Urgency_Report["angular_vel"]+ TRANS_LIN_VEL)


def coordinate_conv():
    print("start")
    global obs_x,obs_y
    for i in range(60,607):
        r=rayToDistance(i)

        if(r<=5.6):
            fie=((240/667)*i)-30
            fie_rad=math.radians(fie)
            #transfromation of coordinates
            x_obs= r*(math.sin(fie_rad))
            y_obs= r*(math.cos(fie_rad))

            imu_rads = imu.getRollPitchYaw()
            p=math.degrees(imu_rads[0])
            # print("p is ",p)
            # print("i is ",i)
            if(p>-180 and p<=-90):
                # print("enter region 1")
                Angle=math.radians(abs(p)-90)
                final_x_obs=(x_obs)*math.cos(Angle) + (y_obs)*math.sin(Angle)+gps.getValues()[0]
                final_y_obs=-(x_obs)*math.sin(Angle) + (y_obs)*math.cos(Angle)+gps.getValues()[1]

            elif(p>90 and p<=180):
                # print("enter region 2")
                Angle=math.radians(180-p)
                # print(math.degrees(Angle))
                final_x_obs=-(x_obs)*math.sin(Angle) + (y_obs)*math.cos(Angle)+gps.getValues()[0]
                final_y_obs=-(x_obs)*math.cos(Angle) - (y_obs)*math.sin(Angle)+gps.getValues()[1]

            elif(p>=0 and p<=90):
                # print("enter region 3")
                Angle=math.radians(p)
                final_x_obs=-(x_obs)*math.sin(Angle) - (y_obs)*math.cos(Angle)+gps.getValues()[0]
                final_y_obs=(x_obs)*math.cos(Angle) - (y_obs)*math.sin(Angle)+gps.getValues()[1]

            elif(p>-90 and p<0):
                # print("enter region 4")
                Angle=math.radians(abs(p))
                final_x_obs=(x_obs)*math.sin(Angle) - (y_obs)*math.cos(Angle)+gps.getValues()[0]
                final_y_obs=(x_obs)*math.cos(Angle) + (y_obs)*math.sin(Angle)+gps.getValues()[1] 

            # print("final x obs is: ",final_x_obs)
            # print("final y obs is : ",final_y_obs)
            # print("x obs is: ",x_obs)
            # print("y obs is : ",y_obs)
            # print("current bot position",gps.getValues()[0],gps.getValues()[1],sep=" ")
            obs_x[i]=final_x_obs
            obs_y[i]=final_y_obs

def waypoint_identifier():
    global new_waypoint,obs_x,obs_y
    new_waypoint.clear()
    for i in range(60,607):
        for j in waypoints:
            diff_x=abs(j[0]-obs_x[i])
            diff_y=abs(j[1]-obs_y[i])
            if(diff_x<0.25 and diff_y<0.25):
                # print("---------------")
                # print("waypoint is :",j[0],j[1], sep=" ")
                # print("diff_x is :",diff_x)
                # print("diff_y is :",diff_y)
                # print("ray index is :",i)
                if j not in new_waypoint:
                    new_waypoint.append(j)
                # print(new_waypoint)
                # print("---------------")

def shortest_waypoint():
    global new_waypoint,closest_waypoint
    min=0
    for i in new_waypoint:
        shortest=math.dist(i,[gps.getValues()[0],gps.getValues()[1]])
        # print("shortest distance is:",shortest)
        if(min==0):
            min=shortest
            closest_waypoint[0]=i[0]
            closest_waypoint[1]=i[1]
        if(min>shortest):
            min=shortest
            closest_waypoint[0]=i[0]
            closest_waypoint[1]=i[1]    

    
    
def goToPOI():
        global n,imu1,targetCoordinate,targetIndex,diffAngle,v,closest_waypoint,gap
        # print(n,"n")
        n=n+1
        targetCoordinate = closest_waypoint
        gap = math.dist(targetCoordinate,[gps.getValues()[0],gps.getValues()[1]])
        

        print(targetCoordinate,"target")
        # print(gps.getValues(),"gps")
        if targetCoordinate[1]-gps.getValues()[1] != 0:
        #considering red axis along bot as refernce axis
            slope=math.atan((targetCoordinate[0]-gps.getValues()[0])/(targetCoordinate[1]-gps.getValues()[1]))
            if targetCoordinate[1]-gps.getValues()[1]>0:
                angle=slope-(math.pi/2)
            elif targetCoordinate[1]-gps.getValues()[1]<0:
                angle=slope+(math.pi/2)    

        if targetCoordinate[1]-gps.getValues()[1]==0:
            if targetCoordinate[0]-gps.getValues()[0]>0:
                angle=0
            elif targetCoordinate([1])-gps.getValues()[1]<0:
                angle=-(math.pi)
        theta = math.degrees(angle)
        # print('enter rotate')
        # print(theta)
        # print('enter pid')
        if(n==1):
            imu1 = imu_rads[0]
        imu_deg = math.degrees(imu1)
        # print(imu_deg , 'initial imu')
        if(theta<90 and theta>-90):
            ang = -90-theta
        if(theta<-90 and theta>-180):
            ang = -theta-90
        if(theta>90 and theta<180):
            ang = 270-theta
        # print(ang ,math.radians(ang) , 'ang')
        # print(imu_rads[0] , 'imu')
        v = imu_rads[0] - math.radians(ang)
        diffAngle = math.degrees(v)
        # n=0
        # print(abs(v),"v")
        # print(diffAngle,"gap angle")           222
        # print(theta , '-theta')
        
        
def wpTracing():
    if abs(v)>=0 and abs(v)<=0.05:
        n=0
        print(v, 'stopping')
        motor_left.setVelocity(+5.0)
        motor_right.setVelocity(+5.0)
        # linearpid()
    else:
        motor_left.setVelocity(1.5*v)
        motor_right.setVelocity(-1.5*v)

def ObstacleAvoidance():
    IdentifyRegions()
    ClearanceTest()
    
    if(Urgency_Report["goBackCount"]==0):
        motor_left.setVelocity(3)
        motor_right.setVelocity(-3)
    else:   
        if(Urgency_Report["act"]):
            Steer()
        else:
            motor_left.setVelocity(5.0)
            motor_right.setVelocity(5.0)                
    

def driving():
    global WP,OA,checkRay,gap,A,waypoints,closest_waypoint
    
    goToPOI()
    if(diffAngle>-115 and diffAngle<115):                                                              #exact value need to Calculate
        checkRay = int(abs(333+(diffAngle*666/240)))
        print(checkRay,"checkRay")
    # if(rayToDistance(checkRay)<1):
        # WP = 0
        # OA=1
    # else:
        # WP=1
        # OA=0
    for i in range(checkRay-70,checkRay+70):
        if(rayToDistance(i)<=SafeLine):
                frontArea.append(i)
        # print(rayToDistance(i))
    print(len(frontArea),"lengthArray")
    if(gap<1):
        print("approaching")                                           #change here
        OA=0
        WP=1
        if(gap<0.3):                                                                       #change here
            print("reached")
            print(closest_waypoint,"removed")
            print("all",waypoints)
            waypoints.remove(closest_waypoint)           
            # A=1
        # A=0
    else:
        if(len(frontArea)<=10):                
            WP=1
            OA=0
        else:            
            OA = 1
            WP=0       
    frontArea.clear()   


def waypoint_finder():
    global A,WP,OA,new_waypoint,closest_waypoint,B
    # goToPOI()
    print("enter waypoint checker")
    coordinate_conv()
    waypoint_identifier()
    print("new waypoint is:",new_waypoint)
    shortest_waypoint()
    print("closest waypoint is: ",closest_waypoint)
    # A=0
    if(len(new_waypoint)!=0):
        print("waypoint is there")
        WP=1
        OA=0
        B=1
    else:
        print("apply obstacle avoidance")
        WP=0
        OA=1
        B=0







while robot.step(timestep) != -1:
    point=l1.getPointCloud()
    imu_rads = imu.getRollPitchYaw()
    # print(l1.getMinRange())
    xf=gps.getValues()[0]
    yf=gps.getValues()[1]

    # if(math.dist(waypoints[9],[xf,yf]<=0.1):
        # motor_left.setVelocity(0)
        # motor_right.setVelocity(0)
    # else:   
    if(timeCount >240 or timeCount==0):
        diff=math.dist([xi, yi], [xf, yf]) 
        xi=gps.getValues()[0]
        yi=gps.getValues()[1]
        timeCount=0
    print("diff",diff)
    if(diff<0.001):                    #changed from 0.0005 to 0.001
        print("bot is stuck")
        motor_left.setVelocity(-1)
        motor_right.setVelocity(-1)
    
    else:
        if(A==1):
            waypoint_finder()
        if(B==1):
            driving()
        # ObstacleAvoidance()
        print(WP,OA)
        if(OA):
            print("Change to OA")
            ObstacleAvoidance()
            A=1
        if(WP):
            print("allWP",waypoints)
            print("Change to WP")
            wpTracing()
            print(gap,"gap")
    timeCount+=1     
       
