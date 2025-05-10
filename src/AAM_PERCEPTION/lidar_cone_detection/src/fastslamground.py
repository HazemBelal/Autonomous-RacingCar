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
N_PARTICLE = 150  # number of particle
NTH= 150/1.5
class Cone:
    def __init__(self, x,y,color,detectedtime):
        self.x=x
        self.y=y
        self.color=color
        self.detectedtime=detectedtime


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
        self.Q_cov = np.diag([1,1])
        self.keys=[0,0]
        self.currtpred=rospy.Time.now().to_sec()
        self.currtyaw=rospy.Time.now().to_sec()
        self.angular_vel=0
        self.currenttime=rospy.Time.now().to_sec()
        self. path_msg = Path()       
        self. path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = 'map'
        
        self.timefromlast=rospy.Time.now().to_sec()
        self.ans=0
        self.locations=[]
        self.cones_sub = rospy.Subscriber("converted_pc", MarkerArray, self.cones_callback)

        self.commandcontrol_sub = rospy.Subscriber("vel_ekf",Float32MultiArray, self.control_callback)
        self.path_publisher = rospy.Publisher('/robot_path', Path, queue_size=10)
        
        self.pc_pub = rospy.Publisher("cone_loc", MarkerArray, queue_size=1)
        self.testcone=rospy.Publisher("/test", MarkerArray, queue_size=1)
        self.particles_pub = rospy.Publisher("particles_pub", MarkerArray, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_loc', Marker, queue_size=10)
        self.id1=0 
        self.id2=100
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
                            self.ans=controls.data[3]
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
                self.timefromlast=rospy.Time.now().to_sec()
            else:
                currenttime1=rospy.Time.now().to_sec()           
                dt=(currenttime1-self.currenttime)
                for i in range(len(self.particles)):
                    self.particles[i]=self.bicyclemodel(self.particles[i],dt)
            #print(dt)        
            self.currenttime=rospy.Time.now().to_sec()
            self.update()
    

    def update(self):
        #calculating estimate    
        self.est=self.calcest()
        
        cones=self.currentnewcones

        #find oldcones that was detected before
        #find new cones that was n't
        #oldcones contain list with sensorx,sensory ,worldx ,worldy

        newcones,oldcones=self.cones_association (cones)
        #print(newcones)
        #if(len(newcones)>0):
             #for i in range (len(self.cones)):
                #print(self.cones[i].x,self.cones[i].y,self.cones[i].color)
             #print("...............................................")

        #find closest cone in old cones detected


        closestcone=self.findclosest(oldcones)
        #print(closestcone)
        if(len(oldcones)>0):
             
            for i in range(len(self.particles)):
                #list containing measured x measured y pred x pred y
                tocalcweight=self.finndclosestandpred(self.particles[i],closestcone)
                print(tocalcweight)
                #calculating particle weight
                self.particles[i].w=self.calculate_particle_weight(tocalcweight[2],tocalcweight[3],tocalcweight[0],tocalcweight[1])
                #print(self.particles[i].w)
                #print(self.particles[i].x)
                #print(self.particles[i].y)
                #print(".........................")
            self.particles = self.normalize_weight(self.particles)    
            self.particles=self.resampling(self.particles)
            #maxw=self.particles[0].w
            #maxpos=(self.particles[0].x,self.particles[0].y)
            #minw=self.particles[0].w
            #minpos=(self.particles[0].x,self.particles[0].y)
            #idx2=0
            #for i in range(len(self.particles)):
                 #if(self.particles[i].w>maxw):
                      #maxw=self.particles[i].w
                      #maxpos=(self.particles[i].x,self.particles[i].y)
                      #idx2=i
            #for i in range(len(self.particles)):
                #if(self.particles[i].w<minw):
                    #minw=self.particles[i].w
                    #minpos=(self.particles[i].x,self.particles[i].y)
            #print(idx2)       
            #print(maxw)
            #print(maxpos)
            #print(minw)
            #print(minpos)
            #print("...............................................")
        #calculating estimate    
        self.est=self.calcest()


        newcones,oldcones=self.cones_association (cones)  
        #updating main map
        self.cones=self.calconesmap(newcones)
        #updating particles map
        self.particles=self.particlesmap(newcones) #to be 
        #self.visualize(self.cones)
        #print(self.est)
        #print([self.vx,self.vy])
        
        #for i in range (len(self.cones)):
            #print(self.cones[i].x,self.cones[i].y,self.cones[i].color)

        self.visualizec(self.cones)
        x1=self.est[0]
        y1=self.est[1]
        loopclosureflag=self.visualizeb(x1,y1)
        self.visualizepar(self.particles)
        self.visualizeloc()
        
        if(loopclosureflag and (rospy.Time.now().to_sec()-self.timefromlast)>10):
             self.timefromlast=rospy.Time.now().to_sec()
             print("lafffffffffffffffffffffffffffffffffffffffffeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeena")
             marker_array = MarkerArray()
             for point in self.cones:
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

             self.testcone.publish(marker_array)
             
        self.keys=[0,0]
    def visualizeloc(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.est[0]
        marker.pose.position.y = self.est[1]
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green color
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()

        self.marker_pub.publish(marker)
    






    def visualizepar(self,particles):
          marker_array = MarkerArray()
          for particle in self.particles:

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
            marker.id=self.id2
            marker.pose.position.x = (particle.x)
            marker.pose.position.y = (particle.y)
            marker.pose.position.z = 0
            marker.lifetime=rospy.Time(1/5)
            self.id2+=1
            # Add the marker to the marker array
            marker_array.markers.append(marker)
          self.particles_pub.publish(marker_array)
         
    def visualizeb(self,x1,y1):
         pose = PoseStamped()
         loopclosureflag=False
         pose.header.stamp = rospy.Time.now()
         pose.header.frame_id = 'map'
         pose.pose.position.x = x1
         pose.pose.position.y = y1
         pose.pose.position.z = 0
         self.path_msg.poses.append(pose)
         distancefromstart=math.sqrt(x1**2+y1**2)
         print(distancefromstart)
         
         if distancefromstart<2:
              loopclosureflag=True
         self.path_publisher.publish(self.path_msg)
         return loopclosureflag 
    def visualizec(self,cones):
         marker_array = MarkerArray()
         for point in self.cones:
            if(point.color=="blue"):
                 color=[0,0,200]
            else:
                 color= [200,200,0]
        # Set the marker position to the point position
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 255
            marker.color.g=0
            marker.color.b=0
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
                  measuredpositionofconex=self.particles[i].x+rangeofdetected*math.cos(self.particles[i].yaw+ anglebetcarcone)
                  measuredpositionofconey=self.particles[i].y+rangeofdetected*math.sin(self.particles[i].yaw+ anglebetcarcone)
                  self.particles[i].cones.append(Cone(measuredpositionofconex,measuredpositionofconey,cone1[4],0))        
         return self.particles        
    def calconesmap(self,newcones):
         for cone1 in newcones:
              self.cones.append(Cone(cone1[2],cone1[3],cone1[4],rospy.Time.now().to_sec() ))              
         return self.cones
         
    def calcest(self):
         self.est=[0,0,0]
         for i in range(len(self.particles)):
              self.est[0]+=self.particles[i].x*self.particles[i].w
              self.est[1]+=self.particles[i].y*self.particles[i].w
              self.est[2]+=self.particles[i].yaw*self.particles[i].w   
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
              
              if(dist<mindist ):
                   mindist=dist

                   xpred=(conepar.x-particle.x)
                   ypred=(conepar.y-particle.y)
                   r=math.sqrt(xpred*xpred+ypred*ypred)
                   citaexpectedtobeobserved=math.atan2(ypred,xpred)-particle.yaw
                   xpred1=r*math.cos(citaexpectedtobeobserved)
                   ypred1=r*math.sin(citaexpectedtobeobserved)
                   tocalcweight=[closestcone[0],closestcone[1],xpred1,ypred1,conepar.x,conepar.y]
         return tocalcweight     
         
    def cones_association (self,cones):
                
                oldcones=[]
                newcones=[]
                for cone in cones.markers:
                    if (cone.pose.position.x>1 and (cone.pose.position.y>-4 and cone.pose.position.y<4) ):
                    #finding position of cones with respect to the world
                        rangeofdetected=((cone.pose.position.x)**2+(cone.pose.position.y)**2)**0.5
                        anglebetcarcone=math.atan2(cone.pose.position.y,cone.pose.position.x)
                        measuredpositionofconex=self.est[0]+rangeofdetected*math.cos(self.est[2]+ anglebetcarcone)
                        measuredpositionofconey=self.est[1]+rangeofdetected*math.sin(self.est[2]+ anglebetcarcone)
                        
                        idx=0
                        f=0

                        
                        for landmark in self.cones:
                            
                            #if distance between two detected cone and cone already detected less than or equal 2 considered same cone
                            u=math.sqrt(((measuredpositionofconex-landmark.x)**2+(measuredpositionofconey-landmark.y)**2))   
                            if(u<=2.5):
                                oldcones.append([cone.pose.position.x,cone.pose.position.y,measuredpositionofconex,measuredpositionofconey])
                                f=1
                                break
                            idx+=1     
                        if(f==0):
                            
                            if ((cone.pose.position.y>0)):
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
         particle.x=particle.x+self.vx*dt
         particle.y=particle.y+self.vy*dt
         #particle.yaw=particle.yaw+self.angular_vel*dt
         particle.yaw=self.ans
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
        particlesrand = np.random.normal(loc=initial_pose, scale=[0.6, 0.6, np.radians(8)], size=(N_PARTICLE, 3))
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