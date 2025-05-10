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
        self.statevector=np.array([[0],[0],[0]])
        self.covmat=np.array([[0.1,0,0],[0,0.1,0],[0,0,0.08726646]])
        self.predictedmean=np.array([])
        self.predictedcov=np.array([])
        self.keys=[0,0]
        self.bicyclelength = 2.6
        self.currentnewcones=MarkerArray()

        self.fx=np.array([[1,0,0],
                              [0,1,0],
                              [0,0,1]])
        
        # EKF state covariance
        self.Cx = np.diag([0.25,0.25, np.deg2rad(5)])
        self.Rx = np.diag([0.00019088498, 0.0045995642**2])
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

                    #intialize Sigma points matrix
                    Sigma_points=self.statevector

                    #getting square root of covariance matrix
                    cov_sqrt=np.linalg.cholesky(self.covmat)
                    #print("after first square root")
                    #print(cov_sqrt)
                    num_cov_sqrt_rows, num_cols = cov_sqrt.shape
                    num_rows, num_cols = self.statevector.shape
                    num_rows_fx,num_cols_fx=self.fx.shape

                    #intialize new mean and new covariance matrix zero values so that other values are to be added
                    predicted_mean=np.zeros((num_rows,1))
                    predicted_cov=np.zeros((num_rows,num_rows))
                    sigma=(0.75**2)*(num_cov_sqrt_rows+3)-num_cov_sqrt_rows
                                        
                    #adding sigma points
                    for i in range (0,num_cov_sqrt_rows):
                        #transpose used to change to column to be added to last mean prediction
                        tobeadded=np.array([cov_sqrt[:,i]]).T                       
                        columntobeaddedtosigma=self.statevector+(math.sqrt(sigma+num_cov_sqrt_rows)*tobeadded)
                        Sigma_points=np.column_stack((Sigma_points,columntobeaddedtosigma))
                     
                    for i in range (0,num_cov_sqrt_rows):
                        #transpose used to change to column to be subtracted from last mean prediction
                        tobesub= np.array([cov_sqrt[:,i]]).T
                        columntobeaddedtosigma=self.statevector-(math.sqrt(sigma+num_cov_sqrt_rows)*tobesub)
                        Sigma_points=np.column_stack((Sigma_points,columntobeaddedtosigma))
                    #print ("first sigma points predict")
                    #print(Sigma_points)     
                    #weights............ to be revised
                    weightato=sigma/(sigma+num_rows)
                    weightati=1/(2*(sigma+num_rows))
                    
                    #predicting
                    currenttime=rospy.Time.now().to_sec()
                    
                    dt=(currenttime-self.currenttime)
                    if (dt>1):
                         dt=0.05
                    for i in range (0,2*num_cov_sqrt_rows+1):
                        #Applying G(x,u),Do not forget assigning dt
                        Sigma_points[0,i]=Sigma_points[0,i]+self.velcon*math.cos(Sigma_points[2,0]+self.angularcon*dt)*dt
                        Sigma_points[1,i]=Sigma_points[1,i]+self.velcon*math.sin(Sigma_points[2,0]+self.angularcon*dt)*dt
                        Sigma_points[2,i]=Sigma_points[2,i]+self.angularcon*dt
                        if (i==0):
                            w=weightato
                        else:
                            w=weightati
                        Sigma_point=np.array([Sigma_points[:,i]]).T  
                    

                
                        predicted_mean=predicted_mean+ (Sigma_point*w)
                    #print(" predicted mean")

                    #print(predicted_mean) 
                    self.currenttime=rospy.Time.now().to_sec()

                    for i in range (0,2*num_cov_sqrt_rows+1):
                        if (i==0):
                           w=weightato
                        else:
                           w=weightati
                        Sigma_point=np.array([Sigma_points[:,i]]).T

                        sh=np.matmul((Sigma_point-predicted_mean),np.transpose((Sigma_point-predicted_mean)))
                        predicted_cov= predicted_cov+(w)*sh
                    numberlandx2=num_rows-3
                    numberofzerocolumnstobeadded=(numberlandx2)-(num_cols_fx-3)
                    for i in range (0,numberofzerocolumnstobeadded):
                        self.fx=np.column_stack((self.fx,np.array([[0],[0],[0]])))          
                    self.predictedmean=predicted_mean
                    #print(self.fx.shape)
                    #print(self.predictedmean.shape)
                    nu=np.matmul(np.transpose(self.fx),self.Cx)
                    nu2=np.matmul(nu,self.fx)
                    #print("nu2")
                    #print(nu2)
                    self.predictedcov=predicted_cov
                    #print("predicted covariance")
                    #print(self.predictedcov)
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
                    measuredpositionofconex=self.predictedmean[0,0]+rangeofdetected*math.cos(self.predictedmean[2,0]+ anglebetcarcone)
                    measuredpositionofconey=self.predictedmean[1,0]+rangeofdetected*math.sin(self.predictedmean[2,0]+ anglebetcarcone)
                    
                    idx=0
                    f=0
                    for landmark in self.landmarks:   
                          if((((measuredpositionofconex-landmark[0])**2+(measuredpositionofconey-landmark[1])**2)**0.5)<=2):
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
          
          #sqrt predicted cov

          cov_sqrt=np.linalg.cholesky(self.predictedcov)
          #print("2nd cov sqrt")
          #print(cov_sqrt)
                
          #intialize sigma points
          Sigma_points=self.predictedmean

          num_cov_sqrt_rows, num_cols = cov_sqrt.shape

          rows,cols=self.predictedmean.shape
         

          sigma=(0.75**2)*(rows+3)-rows


          #intialize new mean and new covariance matrix zero values so that other values are to be added
          hx_mean=np.zeros((2,1))
          hx_cov=np.zeros((2,2))
          usedcov=np.zeros((rows,2)) #msh fahem deeh eh bs leeha dor

          #first part of sigma 
          for i in range (0,rows):
                #transpose used to change to column to be added to last mean prediction
                 tobeadded=(np.array([cov_sqrt[:,i]])).T 
                 columntobeaddedtosigma=self.predictedmean+(math.sqrt(sigma+rows)*tobeadded)
                 Sigma_points=np.column_stack((Sigma_points,columntobeaddedtosigma))
          #second part
          for i in range (0,rows):
                #transpose used to change to column to be added to last mean prediction
                tobesub= (np.array([cov_sqrt[:,i]])).T
                columntobeaddedtosigma=self.predictedmean-(math.sqrt(sigma+rows)*tobesub)
                Sigma_points=np.column_stack((Sigma_points,columntobeaddedtosigma))
          #print ("2nd sigma points up")
          #print(Sigma_points)    
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
                for i in range (0,2*rows+1):
                    xlandmarkidx=3+(idxinstate)*2
                    ylandmarkidx=3+(idxinstate)*2+1
                    xcar=Sigma_points[0,i]
                    ycar=Sigma_points[1,i]
                    xlandmark=Sigma_points[xlandmarkidx,i]
                    ylandmark=Sigma_points[ylandmarkidx,i]
                    q=math.sqrt((xlandmark-xcar)**2+(ylandmark-ycar)**2)
                    anglepred=math.atan2((ylandmark-ycar),(xlandmark-xcar))-Sigma_points[2,i]
                    tobeaddedtohxresult=np.array([[q],[anglepred]])
                    if(i==0):
                        hxresult=tobeaddedtohxresult
                    else:                
                        hxresult=np.column_stack((hxresult,tobeaddedtohxresult))
                
          else:
               hxresult=np.zeros((2, 2*rows+1))
               zsensor=np.array([[0],[0]])
          #print("hx")
          #print(hxresult)        
          
          weightato=sigma/(sigma+rows)
          weightati=1/(2*(sigma+rows))

          #mean of H(X)
          for i in range (0,2*rows+1):
                if (i==0):
                    w=weightato
                else:
                    w=weightati

                hxresult2=(np.array([hxresult[:,i]])).T
                hx_mean=hx_mean+ (hxresult2*w)

        
          for i in range (0,2*rows+1):
                if (i==0):
                    w=weightato
                else:
                    w=weightati

                hxresult2=(np.array([hxresult[:,i]])).T 
               
                hx_cov= hx_cov+(w)*((hxresult2-hx_mean)@ (hxresult2-hx_mean).T)
          print("hx")
          print(hx_mean)
          print("sensor result")
          print(zsensor)
         
          
          S=hx_cov+self.Rx

          #print("s")
          #print(S)

          for i in range (0,2*num_cov_sqrt_rows+1):
                if (i==0):
                    w=weightato
                else:
                    w=weightati

                Sigma_point=(np.array([Sigma_points[:,i]])).T

                hxresult2=(np.array([hxresult[:,i]])).T

                u=((Sigma_point)-self.predictedmean)
                n=np.transpose(hxresult2-hx_mean)

          
                usedcov= usedcov+(w)*(u @ n)

          #print("used cov")
          #print(usedcov)
          
                

          


          kalmangain=usedcov @ np.linalg.inv(S)
          
          #print("Kalman gain")
          #print(kalmangain)
          

          self.statevector=self.predictedmean+kalmangain @ (hx_mean-zsensor)

          self.statevector=self.predictedmean

          u=np.matmul(kalmangain,S)
          
          n=np.transpose(kalmangain)
          

          self.covmat = self.predictedcov - (kalmangain @ S @ kalmangain.T)
          #self.covmat=self.predictedcov

          print("self predicted mean")
          print(self.predictedmean)

        
          


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
                 self .predictedmean =np.row_stack((self.predictedmean,[newcones[i][0]]))
                 self .predictedmean =np.row_stack((self.predictedmean,[newcones[i][1]]))
                
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.00019332663
                 num_cov_sqrt_rows, num_cols = self.predictedcov.shape
                 col=np.zeros((num_cols,1))
                 row=np.zeros((1,num_cols+1))
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.00019332663


                                
         else:
              
              for i in range (0 ,len(newcones)):
                 num_cov_sqrt_rows, num_cols = self.predictedcov.shape

                 col=np.zeros((num_cols,1))
                 row=np.zeros((1,num_cols+1))
                 self.landmarks.append(newcones[i])
                 self .predictedmean =np.row_stack((self.predictedmean,[newcones[i][0]]))
                 self .predictedmean =np.row_stack((self.predictedmean,[newcones[i][1]])) 
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.00019332663
                 num_cov_sqrt_rows, num_cols = self.predictedcov.shape
                 col=np.zeros((num_cols,1))
                 row=np.zeros((1,num_cols+1))
                 self.predictedcov=np.column_stack((self.predictedcov,col))
                 self.predictedcov=np.row_stack((self.predictedcov,row))
                 self.predictedcov[num_cols,num_cols]=0.00019332663

              
if __name__ == '__main__':
    try:
        ukalman = ukalmanFilter()
        del ukalman
    except rospy.ROSInterruptException:
        pass
    rospy.spin()