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
from geometry_msgs.msg import PoseStamped, Quaternion
from ma_rrt_path_plan.msg import WaypointsArray

# Increase particle count for better diversity
N_PARTICLE = 50  # number of particle
# Resample when the effective number of particles drops below this threshold
NTH = 10
class Cone:
    def __init__(self, x,y,color,coneid):
        self.x=x
        self.y=y
        self.color=color
        self.coneid=coneid
        

class Particle:

    def __init__(self, x,y,yaw):
        self.w = 1.0 / N_PARTICLE
        self.x = x
        self.y = y
        self.yaw = yaw
        # landmark x-y positions
        self.lm = np.zeros((500, 2))
        # landmark position covariance
        self.lmP = np.zeros((500 * 2, 2))
        # landmark position covariance
        self.cones=[]
class fastslam:
    def __init__(self,particles):
        rospy.init_node("uKalman_Filter_node", anonymous = True)
        rospy.loginfo("Initializing fastslam with %d particles", len(particles))
        self.particles=particles
        self.loopclosures=0
        self.est=[0,0,0]
        self.goodcones=[]
        self.faultycones=[]
        self.vx=0
        self.vy=0
        self.yawspeed=0
        self.yawprev=0
        # Reduced process noise to better reflect sensor uncertainty and
        # produce smoother particle motion
        self.Q_cov = np.diag([0.05, np.deg2rad(5.0)]) ** 2
        # Cached cones for the next measurement update
        self.currentnewcones = None
        self.currtpred=rospy.Time.now().to_sec()
        self.currtyaw=rospy.Time.now().to_sec()
        self.angular_vel=0
        self.last_pred_time = rospy.Time.now().to_sec()
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        # Use the global map frame for the published path so that RViz can
        # correctly display it without requiring a transform from 'odom'.
        self.path_msg.header.frame_id = 'map'
        self.currentconeid=0
        self.timefromlast=rospy.Time.now().to_sec()
        self.ans=0
        self.locations=[]
        self.cones_sub = rospy.Subscriber("/camera_cones_marker", MarkerArray, self.cones_callback)
        rospy.Subscriber('/waypoints', WaypointsArray, self.waypoints_callback)

        self.commandcontrol_sub = rospy.Subscriber("vel_ekf",Float32MultiArray, self.control_callback)
        self.path_publisher = rospy.Publisher('/robot_path', Path, queue_size=1)
        
        self.pc_pub = rospy.Publisher("cone_loc", MarkerArray, queue_size=1)
        self.testcone=rospy.Publisher("/test", MarkerArray, queue_size=1)
        self.particles_pub = rospy.Publisher("particles_pub", MarkerArray, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_loc', Marker, queue_size=10)
        self.id1=0
        self.id2=100
        # Run prediction at a fixed rate so dt remains consistent
        self.pred_timer = rospy.Timer(rospy.Duration(0.05), self._on_timer)
    def waypoints_callback(self, wp):
        # Assuming wp has fields 'loopClosure' and 'preliminaryLoopClosure'
        rospy.loginfo("Waypoints callback: loopClosure=%s, preliminary=%s", wp.loopClosure, wp.preliminaryLoopClosure)
        self.loopClosureFlag = wp.loopClosure
        self.preliminaryLoopClosureFlag = wp.preliminaryLoopClosure

    def cones_callback(self,reccones):
        rospy.loginfo("Cones callback: received %d markers", len(reccones.markers))
        # Cache the cones for the next measurement update. Prediction now runs
        # on a timer or whenever control data arrives, so we simply store the
        # cones until the next prediction step processes them.
        self.currentnewcones = reccones
    def control_callback (self,controls):
        rospy.loginfo("Control callback: vx=%f vy=%f yaw=%f", controls.data[0], controls.data[1], controls.data[3])
        self.vx = controls.data[0]
        self.vy = controls.data[1]
        self.ans = controls.data[3]

    def _on_timer(self, event):
        """Periodic prediction step run at a fixed frequency."""
        now = rospy.Time.now().to_sec()
        dt = now - self.last_pred_time
        self.last_pred_time = now
        rospy.loginfo("Timer dt computed: %f", dt)
        for i in range(len(self.particles)):
            self.particles[i] = self.bicyclemodel(self.particles[i], dt)

        if self.currentnewcones is not None:
            self.update()
            self.currentnewcones = None
    def update(self):
        rospy.loginfo("Update step started")
        #calculating estimate
        self.est=self.calcest()
        rospy.loginfo("Estimated pose: x=%f y=%f yaw=%f", self.est[0], self.est[1], self.est[2])
        
        cones=self.currentnewcones

        #find oldcones that was detected before
        #find new cones that was n't
        #oldcones contain list with sensorx,sensory ,worldx ,worldy

        newcones,oldcones=self.cones_association (cones,self.goodcones,0)
        rospy.loginfo("Cone association: %d new, %d old", len(newcones), len(oldcones))
        #print (oldcones)
        #print("newcones")
        #print(newcones)
        #print("Faulty Cones")
        #for i in range (len(self.faultycones)):
            #print(self.faultycones[i].x,self.faultycones[i].y,self.faultycones[i].coneid)
        #print(oldcones)

        #print(newcones)
        #if(len(newcones)>0):
             #for i in range (len(self.cones)):
                #print(self.cones[i].x,self.cones[i].y,self.cones[i].color)
             #print("...............................................")
        if(len(oldcones)>0):    
            for oldcone in oldcones:
                for i in range(len(self.particles)):
                    w = self.compute_weight(self.particles[i], oldcone)
                    self.particles[i].w *= w
                    self.particles[i] = self.update_landmark(self.particles[i], oldcone)
        #print(closestcone)

        #print(self.particles[0].lm)
        #print(self.particles[1].lm)
        #print(self.particles[2].lm)
        #print(self.particles[3].lm)
        #print(self.particles[4].lm)
        if(self.loopclosures==0):
            self.faultycones=self.calconespart(newcones)
        self.particles = self.normalize_weight(self.particles)
        self.particles=self.resampling(self.particles)
        self.est=self.calcest()


        
        #updating main map
        if(self.loopclosures==0):
            newcones,oldcones=self.cones_association (cones,self.goodcones,1) 
            self.goodcones=self.calconesmap(newcones)
        if(self.loopclosures==0):
            for newcone in newcones:
                for i in range(len(self.particles)):
                    self.particles[i]=self.add_new_landmark(self.particles[i],newcone)
                #print(Particles[i].x,Particles[i].y)    
        #self.visualize(self.cones)
        #print(self.est)
        #print([self.vx,self.vy])
        
        #for i in range (len(self.cones)):
            #print(self.cones[i].x,self.cones[i].y,self.cones[i].color)

        self.visualizec(self.goodcones)
        x1=self.est[0]
        y1=self.est[1]
        loopclosureflag=self.visualizeb(x1,y1)
        self.visualizepar(self.particles)
        self.visualizeloc()
        rospy.loginfo("Time since last loop closure %.2f", (rospy.Time.now().to_sec()-self.timefromlast))
        if(loopclosureflag and (rospy.Time.now().to_sec()-self.timefromlast)>10):
             self.timefromlast=rospy.Time.now().to_sec()
             print("lafffffffffffffffffffffffffffffffffffffffffeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeena")
             self.loopclosures+=1
             marker_array = MarkerArray()
             for point in self.goodcones:
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
             
    def visualizeloc(self):
        rospy.loginfo("Visualizing current location x=%f y=%f", self.est[0], self.est[1])
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
          rospy.loginfo("Visualizing %d particles", len(self.particles))
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
         
    def visualizeb(self, x1, y1):
        rospy.loginfo("Adding pose to path x=%f y=%f", x1, y1)
        # Create a new PoseStamped for the current estimate
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x1
        pose.pose.position.y = y1
        pose.pose.position.z = 0  # or your desired z value
        pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        # Append the current pose to the path message
        self.path_msg.poses.append(pose)
        rospy.loginfo("Path contains %d poses", len(self.path_msg.poses))

        # (Optional) Limit the number of poses to avoid an ever‐growing list
        # max_poses = 1000
        # if len(self.path_msg.poses) > max_poses:
        #     self.path_msg.poses = self.path_msg.poses[-max_poses:]

        # Publish the updated path with the current time stamp
        self.path_msg.header.stamp = rospy.Time.now()

        self.path_publisher.publish(self.path_msg)
        rospy.loginfo("Published path")

        # Instead of computing the loop closure flag locally, use the flag from /waypoints:
        loopclosureflag = self.loopClosureFlag  if hasattr(self, 'loopClosureFlag') else False

        return loopclosureflag

    def visualizec(self,cones):
         rospy.loginfo("Visualizing %d good cones", len(self.goodcones))
         marker_array = MarkerArray()
         for point in self.goodcones:
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
            marker.color.r = color[0]
            marker.color.g=color[1]
            marker.color.b=color[2]
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
    

    def add_new_landmark(self,particle, z):
        rospy.loginfo("Adding new landmark id=%d", int(z[5]))
        r=((z[0])**2+(z[1])**2)**0.5
        b=math.atan2(z[1],z[0])
        lm_id = int(z[5])

        s = math.sin(self.pi_2_pi(particle.yaw + b))
        c = math.cos(self.pi_2_pi(particle.yaw + b))

        particle.lm[lm_id, 0] = particle.x + r * c
        particle.lm[lm_id, 1] = particle.y + r * s

        # covariance
        dx = r * c
        dy = r * s
        d2 = dx**2 + dy**2
        d = math.sqrt(d2)
        Gz = np.array([[dx / d, dy / d],
                    [-dy / d2, dx / d2]])
        particle.lmP[2 * lm_id:2 * lm_id + 2] = np.linalg.inv(
            Gz) @ self.Q_cov @ np.linalg.inv(Gz.T)

        return particle

    def calconesmap(self,newcones):
         rospy.loginfo("Updating good cones map with %d new cones", len(newcones))
         for cone1 in newcones:
              self.goodcones.append(Cone(cone1[2],cone1[3],cone1[4],cone1[5]))
         return self.goodcones
    def calconespart(self,newcones):
         rospy.loginfo("Updating faulty cones with %d new cones", len(newcones))
         for cone1 in newcones:
              self.faultycones.append(Cone(cone1[2],cone1[3],cone1[4],cone1[5] ))
         return self.faultycones   
    def calcest(self):
         rospy.loginfo("Calculating pose estimate")
         self.est=[0,0,0]
         for i in range(len(self.particles)):
              self.est[0]+=self.particles[i].x*self.particles[i].w
              self.est[1]+=self.particles[i].y*self.particles[i].w
              self.est[2]+=self.particles[i].yaw*self.particles[i].w   
         return self.est
    
    def normalize_weight(self,particles):
        rospy.loginfo("Normalizing particle weights")
        sum_w = sum([p.w for p in particles])

        try:
            for i in range(N_PARTICLE):
                particles[i].w /= sum_w
                #print(particles[i].w)
        except :
            for i in range(N_PARTICLE):
                particles[i].w = 1.0 / N_PARTICLE

            return particles

        rospy.loginfo("Resampling complete")
        return particles
    
    def update_landmark (self,particle, z):
        rospy.loginfo("Updating landmark id=%d", int(z[2]))

        lm_id = int(z[2])
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2, :])

        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf)
        r=((z[0])**2+(z[1])**2)**0.5
        b=math.atan2(z[1],z[0])
        zu=[r,b,z[2]]
        zu=np.array(zu)
        dz = zu[0:2].reshape(2, 1) - zp
        dz[1, 0] = self.pi_2_pi(dz[1, 0])

        xf, Pf = self.update_kf_with_cholesky(xf, Pf, dz, Hf)

        particle.lm[lm_id, :] = xf.T
        particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf

        return particle

    
    def resampling(self,particles):
        rospy.loginfo("Resampling particles")
   
        
        


        pw = []
        for i in range(N_PARTICLE):
            pw.append(particles[i].w)

        pw = np.array(pw)
        #print(pw)

        n_eff = 1.0 / (pw @ pw.T)  # Effective particle number
        rospy.loginfo("Effective particles: %f", n_eff)

        if n_eff < NTH:
            rospy.loginfo("Performing resampling")
            #print("resampled")
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

    def compute_weight(self,particle, z):
        rospy.loginfo("Computing weight for landmark id=%d", int(z[2]))
        lm_id = int(z[2])
        r=((z[0])**2+(z[1])**2)**0.5
        b=math.atan2(z[1],z[0])
        
        #print("xf")
        dist=100
        #for i in range(100):
            #xu = np.array(particle.lm[i, :]).reshape(2, 1)
            #if(xu[0,0]!=0 and xu[1,0]!=0):
                #dx = xu[0, 0] - particle.x
                #dy = xu[1, 0] - particle.y
                #d2 = dx ** 2 + dy ** 2
                #d = math.sqrt(d2)
                #d2=d*math.cos(self.pi_2_pi(math.atan2(dy, dx) - particle.yaw))
                #d3=d*math.sin(self.pi_2_pi(math.atan2(dy, dx) - particle.yaw))
                #if((((z[0]-d2)**2+(z[1]-d3)**2)**0.5)<dist):
                    #dist=(((z[0]-d2)**2+(z[1]-d3)**2)**0.5)
                    #lm_id=i
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)             
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf)
        zu=[r,b,z[2]]
        zu=np.array(zu)
        dx = zu[0:2].reshape(2, 1) - zp
        #print(zu)
        #print(zp)
        #print(xf)
        

        #print("..........")
        dx[1, 0] = self.pi_2_pi(dx[1, 0])

        try:
           invS = np.linalg.inv(Sf)
        except np.linalg.linalg.LinAlgError:
            return 1.0

        num = np.exp(-0.5 * (dx.T @ invS @ dx))[0, 0]
        den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

        w = num / den

        return w
    def compute_jacobians(self,particle, xf, Pf):
        rospy.loginfo("Computing Jacobians")
        dx = xf[0, 0] - particle.x
        dy = xf[1, 0] - particle.y
        d2 = dx ** 2 + dy ** 2
        d = math.sqrt(d2)
        #print(particle.x,particle.y)
        zp = np.array(
            [d, self.pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

        Hv = np.array([[-dx / d, -dy / d, 0.0],
                   [dy / d2, -dx / d2, -1.0]])

        Hf = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])

        Sf = Hf @ Pf @ Hf.T + self.Q_cov

        return zp, Hv, Hf, Sf 
    def update_kf_with_cholesky(self,xf, Pf, v, Hf):
        rospy.loginfo("Updating landmark with Kalman filter")
        PHt = Pf @ Hf.T
        S = Hf @ PHt + self.Q_cov

        S = (S + S.T) * 0.5
        s_chol = np.linalg.cholesky(S).T
        s_chol_inv = np.linalg.inv(s_chol)
        W1 = PHt @ s_chol_inv
        W = W1 @ s_chol_inv.T

        x = xf + W @ v
        P = Pf - W1 @ W1.T

        return x, P

         
    def cones_association (self,cones,conestype,partorest):
                rospy.loginfo("Associating %d cones", len(cones.markers))
                
                oldcones=[]
                newcones=[]
                for cone in cones.markers:
                    if ((cone.pose.position.x>1 and  cone.pose.position.x<6) and (cone.pose.position.y>-4 and cone.pose.position.y<4) ):
                    #finding position of cones with respect to the world
                        rangeofdetected=((cone.pose.position.x)**2+(cone.pose.position.y)**2)**0.5
                        anglebetcarcone=math.atan2(cone.pose.position.y,cone.pose.position.x)
                        measuredpositionofconex=self.est[0]+rangeofdetected*math.cos(self.est[2]+ anglebetcarcone)
                        measuredpositionofconey=self.est[1]+rangeofdetected*math.sin(self.est[2]+ anglebetcarcone)
                        
                        idx=0
                        f=0

                        
                        for landmark in conestype:
                            
                            #if distance between two detected cone and cone already detected less than or equal 2 considered same cone
                            u=math.sqrt(((measuredpositionofconex-landmark.x)**2+(measuredpositionofconey-landmark.y)**2))   
                            if(u<=2.8):
                                oldcones.append([cone.pose.position.x,cone.pose.position.y,landmark.coneid])
                                f=1
                                break
                            idx+=1     
                        if(f==0):
                            
                            if ((cone.color.r==0)and (cone.color.g==0) and (cone.color.b==200) ):
                                color="blue"
                            else:
                                color="yellow"         
                            newcones.append([cone.pose.position.x,cone.pose.position.y,measuredpositionofconex,measuredpositionofconey,color,self.currentconeid])
                            if(partorest==1):
                                self.currentconeid+=1
        
                return newcones,oldcones   
    def bicyclemodel(self,particle,dt):
         rospy.loginfo("Bicycle model dt=%f", dt)
         # Smaller noise reflects the filtered state estimation
         mean = 0
         std_dev = 0.2
         noise = np.random.normal(mean, std_dev)

         # Transform body frame velocities to the world frame using the
         # orientation estimate. This significantly reduces the zigzag
         # behaviour caused by applying body frame velocities directly.
         yaw = self.ans
         vx_w = self.vx * math.cos(yaw) - self.vy * math.sin(yaw)
         vy_w = self.vx * math.sin(yaw) + self.vy * math.cos(yaw)

         particle.x = particle.x + (vx_w + noise) * dt
         particle.y = particle.y + (vy_w + noise) * dt
         particle.yaw = self.pi_2_pi(yaw)

         return particle
    def pi_2_pi(self,angle):
       #normalize angle
       return (angle + math.pi) % (2 * math.pi) - math.pi                         
if __name__ == '__main__':
    rospy.loginfo("Starting fastslam node")
    try:
        # Known starting position
        x_range = (-1, 1)  # Range for x
        y_range = (-1, 1)  # Range for y
        theta_range = (-np.pi, np.pi)  # Range for theta

        # Generate particles
        particlesrand = np.random.uniform(
            low=[x_range[0], y_range[0], theta_range[0]], 
            high=[x_range[1], y_range[1], theta_range[1]], 
            size=(N_PARTICLE, 3)
        )

        # Calculate the mean of the particles
        mean_particle = np.mean(particlesrand, axis=0)

        # Adjust particles to have the mean of (0, 0, 0)
        particlesrand -= mean_particle 
        Particles=[]

        for i in range(len(particlesrand)):
            X=particlesrand[i][0]
            Y=particlesrand[i][1]
            Yaw=particlesrand[i][2]
            particle=Particle(x=X,y=Y,yaw=Yaw)
            Particles.append(particle)
        rospy.loginfo("Spawned %d initial particles", len(Particles))
        x=fastslam(Particles)

    except rospy.ROSInterruptException:
        pass
    rospy.spin()
