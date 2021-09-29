
from controller import Robot,Lidar
import math
# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Obtain waypoints
waypoints = []
waypoints_string = robot.getCustomData()
waypoints_split = waypoints_string.split()
for i in range(10):
    waypoints_element = [float(waypoints_split[2*i]), float(waypoints_split[2*i+1])]
    waypoints.append(waypoints_element)
print('Waypoints:', waypoints)

# begin{please do not change}
if (len(waypoints_split) != 20):
    waypoints_string = ' '.join(waypoints_split[:20])
# end{please do not change}

# Initialize devices
motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')
arm_1 = robot.getDevice('arm_1_joint')
arm_2 = robot.getDevice('arm_2_joint')
arm_3 = robot.getDevice('arm_3_joint')
arm_4 = robot.getDevice('arm_4_joint')
gps = robot.getDevice('gps')
gps_ee = robot.getDevice('gps_ee')
imu = robot.getDevice('inertial unit')

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
l1 = robot.getDevice('lidar_tilt')
arm_1.setPosition(90.0*3.14159/180.0)
arm_2.setPosition(45.0*3.14159/180.0)

arm_4.setPosition(90.0*3.14159/180.0) # -15.0 or 45.0

gps.enable(timestep)
gps_ee.enable(timestep)
imu.enable(timestep)
l1.enable(timestep)
l1.enablePointCloud()

diff = 1

arr_left_edge = []
arr_right_edge = []

xi = 0
yi = 0

timeCount = 0
OBSTACLE_DIST = 0.8
IndexArray = []

Regions_Report = {
                        "front_C":[], "front_L":[], "left_R":[],
                        "left_C":[], "right_C":[], "right_L":[], "front_R":[],
                    }

REGIONS = [
                "front_C", "front_L", "left_R",
                "left_C", "right_C", "right_L", "front_R",
            ]

iter=0
vel = 0
class DynWallFoll:


    def __init__(self,final,m,dynamic_angle, m_ray ,lidar_ray,sec_near , ang_vel,angle,calc_angle_for_once , imu,checkwallfollow , checkrotate ,dynamic_array,sorting , index_target , index_count,check_edge_mov):  
        self.target = final
        self.n = m
        self.diffAngle = dynamic_angle
        self.n_ray = m_ray
        # self.a = ang_pid_allow
        # self.y = arr
        # self.obs_x = coord_x_arr
        # self.obs_y = coord_y_arr
        self.checkray = lidar_ray
        self.k = sec_near
        self.phi_obs = angle
        # self.move_ray = move_ray
        self.v = ang_vel
        self.oa = calc_angle_for_once
        self.imu_deg = imu
        self.check_wallfollow = checkwallfollow
        self.check_rotate = checkrotate
        self.frontArea = dynamic_array
        self.checkSort = sorting
        self.index = index_target
        self.count = index_count
        self.check_edge = check_edge_mov
    
    def rayToDistance(self,ray):
        px = point[ray].x
        py = point[ray].z

        dist = math.dist([px,py],[0,0])
        return dist

    def check(self):
        # print(len(self.frontArea) , 'size of dynamic under check ^^^^^^^^^^^^^^^^^^^^^^^^')
        self.dynamic_region()
        print(math.dist(p,self.targetCoordinate)," DISTANCE FROM TARGET")
        #######################################                         CODE : WHETHER IT REACHED TO TARGET
        if(math.dist(p,self.targetCoordinate)<=1):
            # arm_2.setPosition(45.0*3.14159/180.0)
            # arm_3.setPosition(90.0*3.14159/180.0)
            # arm_4.setPosition(-45.0*3.14159/180.0) # -15.0 or 45.0
            waypoints.remove(self.targetCoordinate)       #########   REMOVED CURRENT TARGET FROM GIVEN ARRAY IF REACHED
            
        # else:
            # arm_2.setPosition(90.0*3.14159/180.0)
            # arm_3.setPosition(90.0*3.14159/180.0)
            # arm_4.setPosition(-45.0*3.14159/180.0) # -15.0 or 45.0
        if(((len(Regions_Report["left_R"])!=0) and (len(Regions_Report["front_L"])!=0) and (len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_R"])!=0) and (len(Regions_Report["right_L"])!=0) and (len(Regions_Report["right_C"])!=0)) or ((len(Regions_Report["left_C"])!=0) and (len(Regions_Report["left_R"])!=0) and (len(Regions_Report["front_L"])!=0) and (len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_R"])!=0) and (len(Regions_Report["right_L"])!=0)) or ((len(Regions_Report["left_C"])!=0) and (len(Regions_Report["left_R"])!=0) and (len(Regions_Report["front_L"])!=0) and (len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_R"])!=0) and (len(Regions_Report["right_L"])!=0) and (len(Regions_Report["right_C"])!=0))):

            print("enter 180 rotation ()()()()()()()()()()()")
            motor_left.setVelocity(+4)
            motor_right.setVelocity(-4)

        if len(self.frontArea) > 0:
                self.check_wallfollow = 1
                self.wallfollowing()
                self.check_rotate = 1

        elif len(self.frontArea) == 0 and self.k == 0:
            print('no obstacle in range ^^^^^^^^^^^^^^^^^^')
            self.check_wallfollow = 0
            self.move()

        elif self.k==1:
            # if self.count == 1:
            
            if self.index < len(self.sorted_points) - 1:
                self.index += 1
            else: 
                print(self.index , "limit crossed +++++++_____+++++++_____+++++")
                self.index = 0
                self.k = 0
                self.wallfollowing()
           

                # self.count = 0
            self.checkSort=1
            print(self.index , "self.index ______________________________________")
            # self.target=self.sorted_points[self.index]
            # print(self.target,'target changed to')
            self.k = 0
            # self.wallfollowing()


            # for i in self.sorted_points:
            #     self.targetCoordinate=i[iter+1]
            #     if(self.diffAngle>-91 and self.diffAngle<91):
            #         self.target = i
            #         break
            #     else

        # if len(self.frontArea) > 0:
        #         self.check_wallfollow = 1
        #         self.wallfollowing()
        #         self.check_rotate = 1

        # elif len(self.frontArea) == 0 and self.k == 0:
        #     print('no obstacle in range ^^^^^^^^^^^^^^^^^^')
        #     self.check_wallfollow = 0
        #     self.move()

        # elif self.k==1:
        #     # if self.count == 1:
        #     #     self.index += 1
        #     self.checkSort=1
        #     print(self.index , "self.index ")
        #     self.k = 0
        #     # self.wallfollowing()
        #     if(len(Regions_Report["front_C"])>0):
        #         self.wallfollowing()
        #     else:
        #         self.target=self.sorted_points[1]

        self.frontArea.clear()
        



    def distan(self ,i):
            # point=l1.getPointCloud()
            f1 = point[i].x
            f2 = point[i].z
            f3 = point[i].y
            # print(f1,f2,f3 , 'LIDAR VALUES' , i)

            psi = abs((333-i)*240/667)
            d1 = math.dist([f1 , f2] , [0,0])
            # print(d1 , 'distance from lidar')
            if d1 == float('inf'):
                d1 = 5.6
            b = 0.20
            d2  = math.sqrt(b**2 + d1**2 + 2*b*d1*(math.cos(math.radians(psi))))
            
            # print(d2 ,i, 'distance from center') 
            return  d2
    



    def angle(self,ray_i):
        # print('calculating angle')
        theta = (240/667)*(ray_i-333) 
        c = self.distan(ray_i)
        d = math.dist([point[ray_i].x,point[ray_i].z] , [0,0])
        if d == float('inf'):
            d = 5.6
        sin_val = math.sin(math.radians(180 - abs(theta)))


        print(c , 'distance from center')
        print(d , 'distance from lidar')
        # print(self.theta , '-theta')
        # print(sin_val , 'sin value')
        # print((d/c)*sin_val , 'inside inverse')
        # print((math.asin((d/c)*sin_val)) , 'inverse sin')
        
        req_angle = math.degrees(math.asin((d/c)*sin_val))
        
        # print(req_angle , '-phi')
        if theta > 0:
            return req_angle
        elif theta == 0 :
            return 0
        else:
            return (-1 * req_angle)
            


    def dynamic_region(self):
            global iter
            self.IdentifyRegions()
            if(self.checkSort):
                self.sorting()
                self.target=self.sorted_points[self.index]
            # global n,imu1,self.targetCoordinate,targetIndex,diffAngle,v,closest_waypoint,gap
            # print(n,"n")
            
            self.targetCoordinate = self.target
            # gap = math.dist(self.targetCoordinate,[gps_vals[0],gps_vals[1]])
            

            print(self.targetCoordinate,"target   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            # print(gps_vals,"gps")
            if self.targetCoordinate[1]-gps_vals[1] != 0:
            #considering red axis along bot as refernce axis
                slope=math.atan((self.targetCoordinate[0]-gps_vals[0])/(self.targetCoordinate[1]-gps_vals[1]))
                if self.targetCoordinate[1]-gps_vals[1]>0:
                    angle=slope-(math.pi/2)
                elif self.targetCoordinate[1]-gps_vals[1]<0:
                    angle=slope+(math.pi/2)    

            if self.targetCoordinate[1]-gps_vals[1]==0:
                if self.targetCoordinate[0]-gps_vals[0]>0:
                    angle=0
                elif self.targetCoordinate([1])-gps_vals[1]<0:
                    angle=-(math.pi)
            theta = math.degrees(angle)
            # print('enter rotate')
            # print(theta)
            # print('enter pid')
            if(self.n==1):
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
            self.v = imu_rads[0] - math.radians(ang)
            self.diffAngle = math.degrees(self.v)
            print(self.diffAngle,"gap angle#############################################") 
            # return v
            # self.n=0
            # print(abs(v),"v")
            # print(theta , '-theta')

            if(self.diffAngle>-73.5 and self.diffAngle<73.5):                                                              #exact value need to Calculate
                self.checkRay = int(333+(self.diffAngle*666/240))
                print(self.checkRay,"self.checkRay")
                self.checkSort=1
                self.index = 0
                for i in range(self.checkRay-62,self.checkRay+62):
                    if(self.distan(i)<=1):
                        self.frontArea.append(i)
            elif ((self.diffAngle> 73.5 and self.diffAngle< 91) or (self.diffAngle> -91 and self.diffAngle< -73.5)):
                    self.checkRay = int(333+(self.diffAngle*666/240))
                    print(self.checkRay,"self.checkRay")
                    self.checkSort=1
                    self.index = 0
                    for i in range(self.checkRay-14,self.checkRay+14):
                        if(self.distan(i)<=1):
                            self.frontArea.append(i)
            else:
                self.k=1
                # self.count = 1
                


            print(len(self.frontArea),'length of dynamic region ###########################################')
            # if len(self.frontArea) > 0:
            #     self.check_wallfollow = 1
            #     self.wallfollowing()
            #     self.check_rotate = 1
            # elif self.k == 1:
            #     self.target = []
            #     print(self.k , 'self.k')
            #     print("obstacle out of range @@@@@@@@@@@@")
            # else:
            #     print('no obstacle in range @@@@@@@@@@@@@@')
            #     self.check_wallfollow = 0
            #     self.move()

            # self.frontArea.clear()





    def rotate_ray(self):
        global vel
        print('enter rotate_ray $#$#$#$#$#$#$#$#$#$###$#$#$#$#$#$#$#$#$')
        v= self.angularPid_ray_rotate()
        # print(v , "angular velocity $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        self.n_ray = 0
        self.oa = 0
        
        
        if abs(v)>=0 and abs(v)<=0.05:
            self.index = 0
            print(v, 'stopping rotation and moving forward')
            self.oa = 1
            d_str = self.rayToDistance(self.checkRay)
            print(d_str,"distan(333) ++++++++++++++++++++")

            
            
            if d_str > 5.6:
                motor_left.setVelocity(10)
                motor_right.setVelocity(10)
            # elif d_str < 0.9:
            #     motor_left.setVelocity(0)
            #     motor_right.setVelocity(0)
            else:
                motor_left.setVelocity(1.5 * d_str)
                motor_right.setVelocity(1.5 * d_str)
            self.check_rotate = 0
            self.n_ray = 1
        else:
            if len(Regions_Report["left_C"]) > 0 or len(Regions_Report["left_R"]) > 0 or len(Regions_Report["right_C"]) > 0  or len(Regions_Report["right_L"]) > 0 :
                vel = self.edge_avoidance()
            if vel > 0.4 and self.check_edge == 1:
                motor_left.setVelocity(vel)
                motor_right.setVelocity(vel/2)
            elif vel > 0.4 and self.check_edge == 0:
                motor_left.setVelocity(vel/2)
                motor_right.setVelocity(vel)
            elif vel < 0.1 :
                print("rotating after edge avoidance &*&*&*&*&*&*&*&*&*")
                motor_left.setVelocity(0.5+3 * v)
                motor_right.setVelocity(0.5- 3 * v)
            else:
                motor_left.setVelocity(2)
                motor_right.setVelocity(2)




    def edge_avoidance(self):
        print('enter edge avoidance')
        for k in Regions_Report["left_C"]:
            # print(k, 'value of k')
            if k < 0.4 :
                arr_left_edge.append(k)
        for k in Regions_Report["left_R"]:
            if k < 0.4 :
                arr_left_edge.append(k)
        for k in Regions_Report["right_C"]:
            if k < 0.4 :
                arr_right_edge.append(k)
        for k in Regions_Report["right_L"]:
            if k < 0.4 :
                arr_right_edge.append(k)
        print(len(arr_left_edge) , 'length of left avoidance array ')
        print(len(arr_right_edge) , 'length of right avoidance array ')
        if len(arr_left_edge) > len(arr_right_edge):
            self.check_edge = 1
            vel_l = len(arr_left_edge)/30
            print(vel_l , 'edge avoidance velocity left')
            arr_left_edge.clear()
            arr_right_edge.clear()
            return vel_l 


            # print(vel_l , 'edge avoidance velocity left')
            # motor_left.setVelocity(vel_l)
            # motor_right.setVelocity(vel_l/2)

        else:
            self.check_edge = 0
            vel_r = len(arr_right_edge)/30
            print(vel_r , 'edge avoidance velocity right')
            arr_left_edge.clear()
            arr_right_edge.clear()
            return vel_r 


            # print(vel_r , 'edge avoidance velocity right')
            # motor_left.setVelocity(vel_r/2)
            # motor_right.setVelocity(vel_r)
        # arr_left_edge.clear()
        # arr_right_edge.clear()



        



    def angularPid_ray_rotate(self):
        print('enter pid')
        if self.n_ray == 1:
            # imu1 = imu_rads[0]
            self.imu_deg =  math.degrees(imu_rads[0])
            print(self.imu_deg , 'initial imu')
        if self.oa == 1:
            self.phi_obs = self.angle(self.checkRay)
        # print(self.phi_obs , 'self.phi_obs *******************')
        ang = self.imu_deg - self.phi_obs
        if ang >-270 and ang < -180:
            ang = abs(180 + ang)
        elif ang < -270:
            ang = 360 + ang 

        # print(ang ,math.radians(ang) , 'ang &&&&&&&&&&&&&&&&&&&&&&&&&&&'
        print(imu_rads[0] , 'imu')
        # rotate_val = abs(imu_rads[0] - math.radians(ang))
        rotate_val = (imu_rads[0] - math.radians(ang))

        return rotate_val




    def move(self):
        # if self.check_rotate == 1:
        #     self.rotate_ray()
        # if(((len(Regions_Report["left_R"])!=0) and (len(Regions_Report["front_L"])!=0) and (len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_R"])!=0) and (len(Regions_Report["right_L"])!=0) and (len(Regions_Report["right_C"])!=0)) or ((len(Regions_Report["left_C"])!=0) and (len(Regions_Report["left_R"])!=0) and (len(Regions_Report["front_L"])!=0) and (len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_R"])!=0) and (len(Regions_Report["right_L"])!=0)) or ((len(Regions_Report["left_C"])!=0) and (len(Regions_Report["left_R"])!=0) and (len(Regions_Report["front_L"])!=0) and (len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_R"])!=0) and (len(Regions_Report["right_L"])!=0) and (len(Regions_Report["right_C"])!=0))):

        #     print("enter 180 rotation ()()()()()()()()()()()")
        #     motor_left.setVelocity(+4)
        #     motor_right.setVelocity(-4)
        # else:
        #     self.rotate_ray()
        self.rotate_ray()
        
        # else:
        #     motor_left.setVelocity(5)
        #     motor_right.setVelocity(5)
        
        

    def wallfollowing(self):

        # if self.checkray <= 333:
        #     self.wall_following_left()
        # else:
        #     self.wall_following_right()
        print(self.rayToDistance(83),"LIDAR 83",self.rayToDistance(583),"LIDAR 583", sep= "   ")
        g=len(Regions_Report["left_R"])+len(Regions_Report["left_C"])
        h=len(Regions_Report["right_L"])+len(Regions_Report["right_C"])
        print(g,"leftWall",h,"rightWall",sep="      ******      ")
        if(g<=h) and (g!=0):
            self.wall_following_left()
        elif(g>h) and (h!=0):
            self.wall_following_right()
        elif g==0:
            self.wall_following_right()
        elif h==0:
            self.wall_following_left()
        # if(self.rayToDistance(83)<=0.8 and self.rayToDistance(666-83)<=0.8):
        #     if(self.rayToDistance(83)>=self.rayToDistance(666-83)):
        #         self.wall_following_left()
        #     else:
        #         self.wall_following_right()

        # else:
        #     if(self.rayToDistance(83)<=self.rayToDistance(666-83)):
        #         self.wall_following_left()
        #     else:
        #         self.wall_following_right()


    def distance_maintainer_left(self):

        global velocity,omega
        p=2
        direction=1
        # dist_front=rayToDistance(333)

        # Determine values for PD control of distance and P control of angle
        for i in range(61,333):
            if(i==61):
                min_index=i
            if self.rayToDistance(i) < self.rayToDistance(min_index):
                min_index = i
        angle_min = math.radians((333-min_index)*(240/666))
        dist_min = self.rayToDistance(min_index)
        wall_dist=0.5
        e = (dist_min - wall_dist)
        dist_front=self.rayToDistance(333)
        
        max_speed=5
        # velocity=6
        if dist_front < wall_dist:
            velocity = 0
        elif dist_front < wall_dist*2:
            velocity = 0.5*max_speed
        elif abs(angle_min) > 1.75:
            velocity = 0.4*max_speed
        else:
            velocity = max_speed
        omega = max(min(direction*(p*e) + (angle_min-((math.pi)/2)*direction), 1.5), -1.5)

    velocity1,omega1=0,0

    def distance_maintainer_right(self):
        global velocity1,omega1
        p=2
        direction=-1
        wall_dist=0.5
        dist_front=self.distan(333)
        
        max_speed=5

        # Determine values for PD control of distance and P control of angle
        for i in range(334,605):
            if(i==334):
                min_index=i
            if self.distan(i) < self.distan(min_index):
                min_index = i
        angle_min = math.radians((333-min_index)*(240/666))
        dist_min = self.distan(min_index)
        e = (dist_min - wall_dist)
        if dist_front < wall_dist:
            velocity1 = 0
        elif dist_front < wall_dist*2:
            velocity1 = 0.5*max_speed
        elif abs(angle_min) > 1.75:
            velocity1 = 0.4*max_speed
        else:
            velocity1 = max_speed
        # velocity1=6
        omega1 = max(min(direction*(p*e) + (angle_min-((math.pi)/2)*direction), 1.5), -1.5)

    def IdentifyRegions(self):
        global Regions_Report
        for i, region in enumerate(REGIONS):
            if(i<=3):
                Regions_Report[region] = [
                        self.distan(x) for x in range(372-78*i,372-78*(i+1),-1)
                                if self.distan(x) <= OBSTACLE_DIST and self.distan(x) != 'inf']
            elif(i>3):
                Regions_Report[region] = [
                        self.distan(x) for x in range(606-78*(i-4),606-78*(i-3),-1)
                                if self.distan(x) <= OBSTACLE_DIST and self.distan(x) != 'inf'] 
        print(len(Regions_Report["front_C"]),"front C")
        print(len(Regions_Report["left_C"]),"left C")
        print(len(Regions_Report["front_L"]),"front L")
        print(len(Regions_Report["left_R"]),"left R")
        print(len(Regions_Report["front_R"]),"front R")
        print(len(Regions_Report["right_L"]),"right L")
        print(len(Regions_Report["right_C"]),"right C")

    def wall_following_left(self):
        print('enter left wallfollowing')
        global velocity,omega,vel
            
        if((len(Regions_Report["front_L"])==0) and (len(Regions_Report["left_R"])==0) and (len(Regions_Report["left_C"])<=39)):
            if len(Regions_Report["left_C"]) > 0 or len(Regions_Report["left_R"]) > 0 or len(Regions_Report["right_C"]) > 0  or len(Regions_Report["right_L"]) > 0 :
                vel = self.edge_avoidance()
            
            if vel > 0.5 and self.check_edge == 1:
                motor_left.setVelocity(vel/2)
                motor_right.setVelocity(vel)
            else:
                motor_left.setVelocity(5/8)
                motor_right.setVelocity(5)

            print("enter outer corner")
        elif((len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_L"])!=0) and (len(Regions_Report["left_R"])!=0) and (len(Regions_Report["left_C"])!=0)):
            if len(Regions_Report["left_C"]) > 0 or len(Regions_Report["left_R"]) > 0 or len(Regions_Report["right_C"]) > 0  or len(Regions_Report["right_L"]) > 0 :
                vel = self.edge_avoidance()
            
            if vel > 0.5 and self.check_edge == 1:
                motor_left.setVelocity(vel)
                motor_right.setVelocity(vel/2)
            else:
                motor_left.setVelocity(5)
                motor_right.setVelocity(5/8) 

            print("enter inner corner")
        else:
            print("move forward")
            self.distance_maintainer_left()
            motor_left.setVelocity(velocity-omega)
            motor_right.setVelocity(velocity+omega)
            print(self.rayToDistance(83),"distance is")
            print(velocity,"velocity is")
            print(omega,"omega is")


    def wall_following_right(self):
        print('enter right wallfollowing')
        global velocity1,omega1,vel
        if((len(Regions_Report["front_R"])==0) and (len(Regions_Report["right_L"])==0) and (len(Regions_Report["right_C"])<=39)):
            if len(Regions_Report["left_C"]) > 0 or len(Regions_Report["left_R"]) > 0 or len(Regions_Report["right_C"]) > 0  or len(Regions_Report["right_L"]) > 0 :
               vel = self.edge_avoidance()

            if vel > 0.5 and self.check_edge == 0:
                motor_left.setVelocity(vel)
                motor_right.setVelocity(vel/2)
            else:
                motor_left.setVelocity(5)
                motor_right.setVelocity(5/8) 

            print("enter outer corner")
        elif((len(Regions_Report["front_C"])!=0) and (len(Regions_Report["front_R"])!=0) and (len(Regions_Report["right_L"])!=0) and (len(Regions_Report["right_C"])!=0)):
            if len(Regions_Report["left_C"]) > 0 or len(Regions_Report["left_R"]) > 0 or len(Regions_Report["right_C"]) > 0  or len(Regions_Report["right_L"]) > 0 :
                vel = self.edge_avoidance()
            
            if vel > 0.5 and self.check_edge == 0:
                motor_left.setVelocity(vel/2)
                motor_right.setVelocity(vel)
            else:
                motor_left.setVelocity(5/8)
                motor_right.setVelocity(5)

            print("enter inner corner")
        else:
            print("move forward")
            self.distance_maintainer_right()
            motor_left.setVelocity(velocity1-omega1)
            motor_right.setVelocity(velocity1+omega1)
            print(self.distan(666-83),"distance is")
            # print(velocity,"velocity is")
            # print(omega,"omega is")
    ######################################################################    SORTING #################
    def sorting(self):
        self.sorted_points = sorted(waypoints, key=lambda e: math.dist(e, p))
        print(self.sorted_points)





DynWallFollObs = DynWallFoll([0,0],1,0,1,0,0,0,0,1,0,0,1,[],1,0,0,0)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    imu_rads = imu.getRollPitchYaw()
    gps_vals = gps.getValues()

    point=l1.getPointCloud()
    # begin{please do not change}
    gps_ee_vals = gps_ee.getValues()
    robot.setCustomData(waypoints_string + ' ' + str(gps_ee_vals[0]) + ' ' + str(gps_ee_vals[1]))
    # end{please do not change}

    # arm_1.setPosition(90.0*3.14159/180.0)
    # arm_2.setPosition(45.0*3.14159/180.0)
    # arm_4.setPosition(45.0*3.14159/180.0) # -15.0 or 45.0

    # print('Hello World from Python!', gps_vals, gps_ee_vals, [x*180.0/3.14159 for x in imu_rads])
    p = [gps_vals[0] , gps_vals[1]]
    q = [gps_ee_vals[0] , gps_ee_vals[1]]
    # print(gps_ee_vals , 'gps_ee')
    # print(math.dist(p,q) , 'distance between both gps ')
    # motor_left.setVelocity(5.0)
    # motor_right.setVelocity(5.0)
    # if gps_ee_vals[0] > 4.5:
    #     break
    
    if(timeCount >=240):

        diff=math.dist([xi, yi], p)
        xi=gps.getValues()[0]
        yi=gps.getValues()[1]
        
        timeCount=0

    print("diff",diff)
    if(diff<0.05): #changed from 0.0005 to 0.001
        print("bot is stuck")
        motor_left.setVelocity(-2)
        motor_right.setVelocity(-2)
    else:
        # if((len(Regions_Report["front_C"]) > 0) and (len(Regions_Report["front_L"]) > 0) and (len(Regions_Report["front_R"]) > 0) and (len(Regions_Report["left_R"]) > 0) and (len(Regions_Report["right_L"]) > 0) ):
        #     print("enter 180 rotation ()()()()()()()()()()()")
        #     motor_left.setVelocity(+4)
        #     motor_right.setVelocity(-4)
        # else:
        #     DynWallFollObs.check()
    
        DynWallFollObs.check()
    timeCount += 1


    

    
    
print('Bye from Python!')



