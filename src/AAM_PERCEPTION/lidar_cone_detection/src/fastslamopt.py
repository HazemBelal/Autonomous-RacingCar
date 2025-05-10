#!/usr/bin/env python3
import math
import string
import matplotlib.pyplot as plt
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from scipy.stats import multivariate_normal
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
N_PARTICLE = 100  # number of particle
NTH= 100/1.5
class Cone:
    def __init__(self, x,y,color):
        self.x=x
        self.y=y
        self.color=color


class Particle:

    def __init__(self, x,y,yaw):
        self.w = 1.0 / N_PARTICLE
        self.x = x
        self.y = y
        self.yaw = yaw
        # landmark x-y positions
        self.lm = []
        # landmark position covariance
        self.cones=[]
class fastslam:
    def __init__(self,particles):
        rospy.init_node("uKalman_Filter_node", anonymous = True)
        self.particles=particles
        self.est=[0,0,0]
        self.cones=[]
        self.vx=0
        self.vy=0
        self.i=0
        self.yawspeed=0
        self.yawprev=0
        self.Q_cov = np.diag([0.2,0.2])
        self.keys=[0,0]
        self.currtpred=rospy.Time.now().to_sec()
        self.currtyaw=rospy.Time.now().to_sec()
        self.angular_vel=0
        self.currenttime=rospy.Time.now().to_sec()
        self. path_msg = Path()       
        self. path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = 'map'

        self.cones_sub = rospy.Subscriber("/converted_pc", MarkerArray, self.cones_callback)
        self.commandcontrol_sub = rospy.Subscriber("/optical_flow",Float32MultiArray, self.control_callback)
        self.angularvel=rospy.Subscriber("/sensor_imu_hector",Imu,self.imu_callback)
        self.path_publisher = rospy.Publisher('/robot_path', Path, queue_size=10)

        self.pc_pub = rospy.Publisher("cone_loc", MarkerArray, queue_size=1)
        self.id1=0
    def imu_callback(self, msg):
        self.angular_vel = msg.angular_velocity.z    
    def cones_callback(self,reccones):
        if self.keys[0] != -1:
            self.keys[0] = 0
            self.currentnewcones=reccones
            
            self.keys[0] = 1
            self.predict()
    def control_callback (self,controls):
                       
                        if self.keys[1] != -1:
                            self.keys[1] = 0
                            self.vx=controls.data[0]
                            self.vy=controls.data[1]
                            #print(self.vx)
                            self.keys[1] = 1
                            self.predict()
    def predict(self):
                                     
        
        #predicting each particle depending on state estimation readings
        if self.keys[0] == 1 and self.keys[1] == 1:

            self.keys = [-1, -1]
            #calculating dt

            if (self.i==0):
                dt=0.05
                self.i=1
            else:
                currenttime1=rospy.Time.now().to_sec()           
                dt=(currenttime1-self.currenttime)
                for i in range(len(self.particles)):
                    self.particles[i]=self.bicyclemodel(self.particles[i],dt)
            print(dt)        
            self.currenttime=rospy.Time.now().to_sec()
            self.update()
    

    def update(self):
        
        cones=self.currentnewcones

        #find oldcones that was detected before
        #find new cones that was n't
        #oldcones contain list with sensorx,sensory ,worldx ,worldy

        newcones,oldcones=self.cones_association (cones)
        #if(len(newcones)>0):
             #for i in range (len(self.cones)):
                #print(self.cones[i].x,self.cones[i].y,self.cones[i].color)
             #print("...............................................")

        #find closest cone in old cones detected


        closestcone=self.findclosest(oldcones)
        if(len(oldcones)>0):
             
            for i in range(len(self.particles)):
                #list containing measured x measured y pred x pred y
                tocalcweight=self.finndclosestandpred(self.particles[i],closestcone)

                #calculating particle weight
                self.particles[i].w=self.calculate_particle_weight(tocalcweight[2],tocalcweight[3],tocalcweight[0],tocalcweight[1])
                #print(self.particles[i].x)
                #print(self.particles[i].y)
            self.particles=self.resampling(self.particles)

        #calculating estimate    
        self.est=self.calcest()
        #updating main map
        self.cones=self.calconesmap(newcones)#to be checked 
        #updating particles map
        self.particles=self.particlesmap(newcones) #to be 
        #self.visualize(self.cones)
        print(self.est)
        print([self.vx,self.vy])
        
        for i in range (len(self.cones)):
            print(self.cones[i].x,self.cones[i].y,self.cones[i].color)

        self.visualizec(self.cones)
        x1=self.est[0]
        y1=self.est[1]
        self.visualizeb(x1,y1)
        self.keys=[0,0]


    def visualizeb(self,x1,y1):
         pose = PoseStamped()
         pose.header.stamp = rospy.Time.now()
         pose.header.frame_id = 'map'
         pose.pose.position.x = x1
         pose.pose.position.y = y1
         pose.pose.position.z = 0
         self.path_msg.poses.append(pose)
         self.path_publisher.publish(self.path_msg)
    def visualizec(self,cones):
         marker_array = MarkerArray()
         for point in cones:
        # Set the marker position to the point position
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.id=self.id1
            marker.pose.position.x = (point.x)
            marker.pose.position.y = (point.y)
            marker.pose.position.z = 0
            marker.lifetime=rospy.Time(1/5)
            self.id1+=1
            # Add the marker to the marker array
            marker_array.markers.append(marker)
    # print(len(marker_array.markers)) 
         self.pc_pub.publish(marker_array)
    

    def particlesmap(self,newcones):
         for cone1 in newcones:
              for i in range (len(self.particles)):
                  
                  rangeofdetected=((cone1[0])**2+(cone1[1])**2)**0.5
                  anglebetcarcone=math.atan2(cone1[1],cone1[0])
                  measuredpositionofconex=self.est[0]+rangeofdetected*math.cos(self.est[2]+ anglebetcarcone)
                  measuredpositionofconey=self.est[1]+rangeofdetected*math.sin(self.est[2]+ anglebetcarcone)
                  self.particles[i].cones.append(Cone(measuredpositionofconex,measuredpositionofconey,cone1[4]))        
         return self.particles        
    def calconesmap(self,newcones):
         for cone1 in newcones:
              rangeofdetected=((cone1[0])**2+(cone1[1])**2)**0.5
              anglebetcarcone=math.atan2(cone1[1],cone1[0])
              measuredpositionofconex=self.est[0]+rangeofdetected*math.cos(self.est[2]+ anglebetcarcone)
              measuredpositionofconey=self.est[1]+rangeofdetected*math.sin(self.est[2]+ anglebetcarcone)
              self.cones.append(Cone(measuredpositionofconex,measuredpositionofconey,cone1[4]))              
         return self.cones
         
    def calcest(self):
         self.est=[0,0,0]
         for i in range(len(self.particles)):
              self.est[0]+=self.particles[i].x*self.particles[i].w
              self.est[1]+=self.particles[i].y*self.particles[i].w
              self.est[2]+=self.particles[i].yaw*self.particles[i].w
         self.est[2]=self.pi_2_pi(self.est[2])     
         return self.est
    
    def normalize_weight(self,particles):
        sum_w = sum([p.w for p in particles])

        try:
            for i in range(N_PARTICLE):
                particles[i].w /= sum_w
        except ZeroDivisionError:
            for i in range(N_PARTICLE):
                particles[i].w = 1.0 / N_PARTICLE

            return particles

        return particles


    def resampling(self,particles):
   

        particles = self.normalize_weight(particles)


        pw = []
        for i in range(N_PARTICLE):
            pw.append(particles[i].w)

        pw = np.array(pw)
        #print(pw)

        n_eff = 1.0 / (pw @ pw.T)  # Effective particle number
    # print(n_eff)

        if n_eff < NTH:
               # resampling
            w_cum = np.cumsum(pw)
            base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
            resample_id = base + np.random.rand(base.shape[0]) / N_PARTICLE

            inds = []
            ind = 0
            for ip in range(N_PARTICLE):
                while (ind < w_cum.shape[0] - 1) \
                        and (resample_id[ip] > w_cum[ind]):
                    ind += 1
                inds.append(ind)
        
            tmp_particles = self.particles[:]
            for i in range(len(inds)):
                particles[i].x = tmp_particles[inds[i]].x
                particles[i].y = tmp_particles[inds[i]].y
                particles[i].yaw = tmp_particles[inds[i]].yaw
                particles[i].cones = tmp_particles[inds[i]].cones
                particles[i].w = 1.0 / N_PARTICLE

        return particles

    def calculate_particle_weight(self,x_pred, y_pred, x_detected, y_detected):
    # Predicted landmark position
        predicted_landmark = np.array([x_pred, y_pred])

    # Measured position from the sensor
        measured_position = np.array([x_detected, y_detected])

    # Calculate the innovation or residual (difference between predicted and measured)
        innovation = measured_position - predicted_landmark

    # Calculate the weight using the multivariate Gaussian distribution
        weight = multivariate_normal.pdf(innovation, mean=None, cov=self.Q_cov)

        return weight


    def finndclosestandpred(self,particle,closestcone):
         tocalcweight=[]
         mindist=100
         for conepar in particle.cones:
              dist=math.sqrt((conepar.x-closestcone[2])**2+(conepar.y-closestcone[3])**2)
              
              if(dist<mindist):
                   mindist=dist
                   xpred=(conepar.x-particle.x)
                   ypred=(conepar.y-particle.y)
                   tocalcweight=[closestcone[0],closestcone[1],xpred,ypred]
         return tocalcweight     
         
    def cones_association (self,cones):
                
                oldcones=[]
                newcones=[]
                for cone in cones.markers:
                    #finding position of cones with respect to the world
                    rangeofdetected=((cone.pose.position.x)**2+(cone.pose.position.y)**2)**0.5
                    anglebetcarcone=math.atan2(cone.pose.position.y,cone.pose.position.x)
                    measuredpositionofconex=self.est[0]+rangeofdetected*math.cos(self.est[2]+ anglebetcarcone)
                    measuredpositionofconey=self.est[1]+rangeofdetected*math.sin(self.est[2]+ anglebetcarcone)
                    
                    idx=0
                    f=0

                    for landmark in self.cones:
                          #if distance between two detected cone and cone already detected less than or equal 2 considered same cone
                             
                          if((((measuredpositionofconex-landmark.x)**2+(measuredpositionofconey-landmark.y)**2)**0.5)<=3):
                             oldcones.append([cone.pose.position.x,cone.pose.position.y,measuredpositionofconex,measuredpositionofconey])
                             f=1
                             
                             break     
                    if(f==0):
                          
                          if ((cone.pose.position.y<0)):
                                color="blue"
                          else:
                                color="red"      
                          newcones.append([cone.pose.position.x,cone.pose.position.y,measuredpositionofconex,measuredpositionofconey,color])                                  
      
                return newcones,oldcones



    def findclosest(self,oldcones):
            mindist=100
            zsensor=[]
            #To find closest cone in oldcones 
            for oldcone in oldcones:
                if((((oldcone[0])**2+(oldcone[1])**2)**0.5)<mindist):
                    mindist=(((oldcone[0])**2+(oldcone[1])**2)**0.5)
                    zsensor=[oldcone[0],oldcone[1],oldcone[2],oldcone[3]]
            return zsensor        
    def bicyclemodel(self,particle,dt):
        #Simulating bicycle model
         self.vx=self.vx*math.cos(particle.yaw)-self.vy*math.sin(particle.yaw)
         self.vy=self.vx*math.sin(particle.yaw)+self.vy*math.cos(particle.yaw)
         particle.x=particle.x+self.vx*dt
         particle.y=particle.y+self.vy*dt
         particle.yaw=particle.yaw+self.angular_vel*dt
         particle.yaw=self.pi_2_pi(particle.yaw) 
         
         return particle
    def pi_2_pi(self,angle):
       #normalize angle
       return (angle + math.pi) % (2 * math.pi) - math.pi                         
if __name__ == '__main__':
    try:
        # Known starting position
        initial_pose = np.array([0, 0, 0])
        # Initialize particles around the known starting position with small random perturbations
        particlesrand = np.random.normal(loc=initial_pose, scale=[0.25, 0.25, np.radians(10)], size=(N_PARTICLE, 3))
        Particles=[]

        for i in range(len(particlesrand)):
            X=particlesrand[i][0]
            Y=particlesrand[i][1]
            Yaw=particlesrand[i][2]
            particle=Particle(x=X,y=Y,yaw=Yaw)
            Particles.append(particle)
        x=fastslam(Particles)

    except rospy.ROSInterruptException:
        pass
    rospy.spin()       