#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import math
class ukalmanFilter():
    def __init__(self):
        rospy.init_node("uKalman_Filter_node", anonymous = True)


        self.cones_sub = rospy.Subscriber("/converted_pc", MarkerArray, self.cones_callback)
        self.commandcontrol_sub = rospy.Subscriber("/robot_control/command",AckermannDriveStamped, self.control_callback)
        self.velcon=0
        self.steeringcon=0
        self.angularcon=0
        self.landmarks=[]
        self.newlandmarks=[]
        self.statevector=np.array([0,0,0])
        self.covmat=np.array([[0.5,0,0],[0,0.5,0],[0,0,10]])
        self.predictedmean=[]
        self.predictedcov=np.array([])
        self.keys=[0,0]
        self.bicyclelength = 2.6
        self.currentnewcones=MarkerArray()

        self.fx=np.array([[1,0,0],
                              [0,1,0],
                              [0,0,1]])
        
        # EKF state covariance
        self.Cx = np.diag([0.25,0.25, np.deg2rad(5)])
        self.Rx = np.diag([0.20, np.deg2rad(10)])
        self.currenttime=rospy.Time.now().to_sec()
    def cones_callback(self,reccones):
            if self.keys[0] != -1:
                self.keys[0] = 0
                self.currentnewcones=reccones
                self.keys[0] = 1
                self.predict()
    def control_callback (self,controls):
                        if self.keys[1] != -1:
                            self.keys[1] = 0
                            self.velcon=controls.drive.speed
                            self.steeringcon=controls.drive.steering_angle
                            self.angularcon= (controls.drive.speed *math.tan(controls.drive.steering_angle))/ self.bicyclelength
                            self.keys[1] = 1
                            self.predict()

    def predict(self):
                if self.keys[0] == 1 and self.keys[1] == 1:
                    self.keys = [-1, -1]
                    #getting new cones
                    cones=self.currentnewcones
                    num_rows= self.statevector.shape[0]
                    num_rows_fx,num_cols_fx=self.fx.shape
                    #intialize Sigma points matrix
                    Sigma_points=self.sigma_points(self.statevector,self.covmat)

                    
                    #predicting
                    currenttime=rospy.Time.now().to_sec()

                    num_rows_sigma, num_cols_sigma = Sigma_points.shape


                    dt=(currenttime-self.currenttime)
                    if (dt>1):
                         dt=0.05
                    for i in range (0,num_cols_sigma):
                        #Applying G(x,u),Do not forget assigning dt
                        Sigma_points[0,i]=Sigma_points[0,i]+self.velcon*math.cos(Sigma_points[2,0]+self.angularcon*dt)*dt
                        Sigma_points[1,i]=Sigma_points[1,i]+self.velcon*math.sin(Sigma_points[2,0]+self.angularcon*dt)*dt
                        Sigma_points[2,i]=Sigma_points[2,i]+self.angularcon*dt
                    self.currenttime=rospy.Time.now().to_sec()
                    
                    predicted_mean, predicted_cov = self.calculate_mean_and_covariance(Sigma_points)
                    

                    numberlandx2=num_rows-3
                    numberofzerocolumnstobeadded=(numberlandx2)-(num_cols_fx-3)
                    numberofzerocolumnstobeadded=(numberlandx2)-(num_cols_fx-3)
                    for i in range (0,numberofzerocolumnstobeadded):
                        self.fx=np.column_stack((self.fx,np.array([[0],[0],[0]])))


                    self.predictedmean=predicted_mean
                    
                    

                    self.predictedcov=predicted_cov + np.matmul(np.matmul(np.transpose(self.fx),self.Cx),self.fx)


                    newcones,oldcones=self.cones_association (cones)
                    
                    self.addlandmarks(newcones)

                    self.update(newcones,oldcones)


    def cones_association (self,cones):
                oldcones=[]
                newcones=[]
                for cone in cones.markers:
                    #finding position of cones with respect to the world
                    rangeofdetected=((cone.pose.position.x)**2+(cone.pose.position.y)**2)**0.5
                    anglebetcarcone=math.atan2(cone.pose.position.y,cone.pose.position.x)
                    measuredpositionofconex=self.predictedmean[0]+rangeofdetected*math.cos(self.predictedmean[2]+ anglebetcarcone)
                    measuredpositionofconey=self.predictedmean[1]+rangeofdetected*math.sin(self.predictedmean[2]+ anglebetcarcone)
                    
                    idx=0
                    f=0
                    for landmark in self.landmarks:   
                          if((((measuredpositionofconex-landmark[0])**2+(measuredpositionofconey-landmark[1])**2)**0.5)<=1.5):
                             oldcones.append([cone.pose.position.x,cone.pose.position.y,landmark[2],idx,rangeofdetected,anglebetcarcone])
                             f=1
                             
                             break     
                          idx+=1
                    if(f==0):
                          
                          if ((cone.pose.position.y<0)):
                                color="blue"
                          else:
                                color="red"      
                          newcones.append([measuredpositionofconex,measuredpositionofconey,color])                                  
      
                return newcones,oldcones
    
    def update (self,newcones,oldcones):
          
          Sigma_points=Sigma_points=self.sigma_points(self.predictedmean,self.predictedcov)


          num_cov_sqrt_rows, num_cols = Sigma_points.shape

          rows,cols=self.predictedmean.shape
         

          sigma=3-num_cov_sqrt_rows


          #intialize new mean and new covariance matrix zero values so that other values are to be added
          hx_mean=np.zeros((2,1))
          hx_cov=np.zeros((2,2))
          usedcov=np.zeros((num_cov_sqrt_rows,2)) #msh fahem deeh eh bs leeha dor
     
          #finding closest old cone
          mindist=100
          for oldcone in oldcones:
            if((((oldcone[0])**2+(oldcone[1])**2)**0.5)<mindist):
                  mindist=(((oldcone[0])**2+(oldcone[1])**2)**0.5)
                  idxinstate=oldcone[3]
                  oldconetobeused=oldcone
                  zsensor=np.array([[mindist],[math.atan2(oldcone[1],oldcone[0])]])



          #function H(X) to sigma
          if (mindist!=100):              
                for i in range (0,2*num_cov_sqrt_rows+1):
                    xlandmarkidx=3+(idxinstate)*2
                    ylandmarkidx=3+(idxinstate)*2+1
                    xcar=Sigma_points[0,0]
                    ycar=Sigma_points[1,0]
                    xlandmark=Sigma_points[xlandmarkidx,0]
                    ylandmark=Sigma_points[ylandmarkidx,0]
                    q=math.sqrt((xlandmark-xcar)**2+(ylandmark-ycar)**2)
                    anglepred=math.atan2((ylandmark-ycar),(xlandmark-xcar))-Sigma_points[2,0]
                    tobeaddedtohxresult=np.array([[q],[anglepred]])
                    if(i==0):
                        hxresult=tobeaddedtohxresult
                    else:                
                        hxresult=np.column_stack((hxresult,tobeaddedtohxresult))
                
          else:
               hxresult=np.zeros((2, 2*num_cov_sqrt_rows+1))
               zsensor=np.array([[0],[0]])
        


          #mean of H(X)
            
          hx_mean, Pyy = self.calculate_mean_and_covariance(hxresult)
          

          S=Pyy+self.Rx                

          usedcov = self.calculate_cross_correlation(self.predictedmean, Sigma_points, hx_mean, hxresult)


          kalmangain=np.matmul(usedcov,np.linalg.pinv(S))

          
          self.statevector=self.predictedmean+np.matmul(kalmangain,(hx_mean-zsensor))



          self.covmat = self.predictedcov - np.matmul(np.matmul(kalmangain,S),np.transpose(kalmangain))

          

          print(np.matmul(usedcov,np.linalg.pinv(S)))
          


          self.keys=[0,0]
          
    def addlandmarks (self,newcones):
         if (len(self.landmarks)!=0):
              
            landmarksgetcolor=self.landmarks

            self.landmarks=[]
            m=0
            i=3
            num_cov_sqrt_rows, num_cols = self.predictedcov.shape

            while(i<len(self.predictedmean[:,0])):

                self.landmarks.append([self.predictedmean[i,0],self.predictedmean[i+1,0],landmarksgetcolor[m][2]])
                m+=1
                i+=2
            for i in range (0 ,len(newcones)):
                 num_cov_sqrt_rows, num_cols = self.predictedcov.shape

                 col=np.zeros((num_cols,1))
                 row=np.zeros((1,num_cols+1))
                 self.landmarks.append(newcones[i])
                 self .predictedmean .append(newcones[i][0])
                 self .predictedmean.append(newcones[i][1])
                
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.1+self.predictedcov[0,0]
                 num_cov_sqrt_rows, num_cols = self.predictedcov.shape
                 col=np.zeros((num_cols,1))
                 row=np.zeros((1,num_cols+1))
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.2+self.predictedcov[1,1]


                                
         else:
              
              for i in range (0 ,len(newcones)):
                 num_cov_sqrt_rows, num_cols = self.predictedcov.shape

                 col=np.zeros((num_cols,1))
                 row=np.zeros((1,num_cols+1))
                 self.landmarks.append(newcones[i])
                 self .predictedmean .append(newcones[i][0])
                 self .predictedmean.append(newcones[i][1])
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.00027877560135316034+self.predictedcov[0,0]
                 num_cov_sqrt_rows, num_cols = self.predictedcov.shape
                 col=np.zeros((num_cols,1))
                 row=np.zeros((1,num_cols+1))
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.0014666964253173215+self.predictedcov[0,0]

    def sigma_points(self, x, P):
        
        '''
        generating sigma points matrix x_sigma given mean 'x' and covariance 'P'
        '''
        
        nx = np.shape(x)[0]
         
        
        sigma=3-nx

        sigma_scale=math.sqrt(sigma+nx)

        
        x_sigma = np.zeros((nx, 2*nx+1))


        print(x_sigma[:, 0])


        x_sigma[:, 0] = x
        
        S = np.linalg.cholesky(P)
        
        for i in range(nx):
            x_sigma[:, i + 1]      = x + (sigma_scale * S[:, i])
            x_sigma[:, i + nx + 1] = x - (sigma_scale * S[:, i])


        return x_sigma
    

    def calculate_mean_and_covariance(self, y_sigmas):
        ydim = np.shape(y_sigmas)[0]

        sigma=3-ydim  
        
        weightato=sigma/(sigma+ydim)
        weightati=1/(2*(sigma+ydim))
        
        # mean calculation
        y = weightato * y_sigmas[:, 0]
        for i in range(1, 2*ydim+1):
            y += weightati * y_sigmas[:, i]
            
        # covariance calculation
        d = (y_sigmas[:, 0] - y).reshape([-1, 1])
        Pyy = weightato * (d @ d.T)
        for i in range(1, 2*ydim+1):
            d = (y_sigmas[:, i] - y).reshape([-1, 1])
            Pyy += weightati* (d @ d.T)
    
        return y, Pyy
    
    def calculate_cross_correlation(self, x, x_sigmas, y, y_sigmas):
        

        xdim = np.shape(x)[0]
        ydim = np.shape(y)[0]

        sigma=3-xdim  
        
        weightato=sigma/(sigma+xdim)
        weightati=1/(2*(sigma+xdim))
        
        n_sigmas = np.shape(x_sigmas)[1]
    
        dx = (x_sigmas[:, 0] - x).reshape([-1, 1])
        dy = (y_sigmas[:, 0] - y).reshape([-1, 1])
        Pxy = weightato * (dx @ dy.T)
        for i in range(1, n_sigmas):
            dx = (x_sigmas[:, i] - x).reshape([-1, 1])
            dy = (y_sigmas[:, i] - y).reshape([-1, 1])
            Pxy += weightati * (dx @ dy.T)
    
        return Pxy
if __name__ == '__main__':
    try:
        ukalman = ukalmanFilter()
        del ukalman
    except rospy.ROSInterruptException:
        pass
    rospy.spin()