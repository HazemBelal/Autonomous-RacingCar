#!/usr/bin/env python3
from std_msgs.msg import Float64
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
pub = rospy.Publisher("/vx", Float64, queue_size=1)
pub2 = rospy.Publisher("/error", Float64, queue_size=1)
pub3 = rospy.Publisher("/vy", Float64, queue_size=1)
pub4 = rospy.Publisher("/vxavg", Float64, queue_size=1)
pub5 = rospy.Publisher("/vyavg", Float64, queue_size=1)
pub6 = rospy.Publisher("/optical_flow", Float32MultiArray, queue_size=1)
testframe=0
idm=0
prevarr=MarkerArray()
avgcountx=0
avgcounty = 0
avgvx=0
avgvy=0
currenttime=0
realspeed=0
prevtime=0
angvel=0
yaw=0
vxprev=0
vyprev=0
avgxlist=[]
avgylist=[]
realspeedx=0
realspeedy=0
def restore(real):
    global realspeedx,realspeedy
    realspeedx=real.twist.twist.linear.x
    realspeedy=real.twist.twist.linear.y

    #print(realspeed)
def angular(msg):
    global angvel,yaw
    angvel=msg.angular_velocity.z
    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
def callback(data):
    arr = Float32MultiArray()
    global testframe,avgxlist,avgylist ,prevarr,idm,rate,avgcountx,avgcounty,avgvx,avgvy,realspeedx,realspeedy,prevtime,vxprev,vyprev
    testframe2=0
    for i in range(0,len(data.markers)):
        disttest=math.sqrt((data.markers[i].pose.position.x*data.markers[i].pose.position.x)+(data.markers[i].pose.position.y*data.markers[i].pose.position.y)+(data.markers[i].pose.position.z*data.markers[i].pose.position.z))
        if(disttest<10.75):
            testframe2=1
    #print(len(data.markers))        
    if (testframe==0 and len(data.markers)>1 and testframe2==1):
        for m in data.markers:
            m.id=idm
            idm=idm+1
        prevarr=data
        prevtime=rospy.Time.now().to_sec()
        testframe=1
    elif(testframe==1 and (len(data.markers))>1 and testframe2==1):
        firstmin=999999
        ans=0
        secondmin=999999
        clos1coneprev=Marker()
        clos1conecurr=Marker()
        coneleftc=0
        conerightc=0
        coneleftp=0
        conerightp=0
        testpassed1=0
        for i in range(0,len(data.markers)):
            distclos=math.sqrt((data.markers[i].pose.position.x*data.markers[i].pose.position.x)+(data.markers[i].pose.position.y*data.markers[i].pose.position.y)+(data.markers[i].pose.position.z*data.markers[i].pose.position.z))
            if(distclos<firstmin):
                ans=i
                firstmin=distclos
                clos1conecurr=data.markers[i]
        ctest=data.markers[ans].pose.position.y
        if(ctest<0):
            coneleftc=1
        else:
            conerightc=1    
        for i in range(0,len(prevarr.markers)):
            distclos2=math.sqrt((prevarr.markers[i].pose.position.x*prevarr.markers[i].pose.position.x)+(prevarr.markers[i].pose.position.y*prevarr.markers[i].pose.position.y)+(prevarr.markers[i].pose.position.z*prevarr.markers[i].pose.position.z))
            if((prevarr.markers[i].pose.position.y)<0):
                coneleftp=1
            else:
               conerightp=1
            if(abs(firstmin-distclos2)<secondmin and ((coneleftp==1 and coneleftc==1)or(conerightc==1 and conerightp==1))):
                     clos1coneprev=prevarr.markers[i]
                     secondmin=abs(firstmin-distclos2)
                     testpassed1=1
                     coneleftp=0
                     conerightp=0


        prevarr=data                                           
        
        if(testpassed1==1):
            print2x=0
            print2y=0
            currenttime=rospy.Time.now().to_sec()
            vx=-((clos1conecurr.pose.position.x-clos1coneprev.pose.position.x)/(currenttime-prevtime))+(angvel*clos1coneprev.pose.position.y)
            vy=-((clos1conecurr.pose.position.y-clos1coneprev.pose.position.y)/(currenttime-prevtime))-(angvel*clos1coneprev.pose.position.x)
            
            
            prevtime=currenttime
            vxans=vx*math.cos(yaw)-vy*math.sin(yaw)
            vyans=vy*math.cos(yaw)+vx*math.sin(yaw)
            vxmsg = Float64()
            vymsg=Float64()
            error=Float64()
            vxmsg.data=vx
            vymsg.data=vy
            vxavg=Float64()
            vyavg=Float64()
            if(abs(vxmsg.data-vxprev)<2):
                pub.publish(vxmsg)
                vxprev=vxmsg.data
                print2x=1
                if(avgcountx<10):
                    avgxlist.append(vxmsg.data)
                    avgcountx=avgcountx+1
                else:
                    vxavg.data=sum(avgxlist)/10.0
                    
                    pub4.publish(vxavg)
                    
                    avgxlist.pop(0)
                    avgxlist.append(vxmsg.data)
            if(abs(vymsg.data-vyprev)<3):
                print2y=1
                pub3.publish(vymsg)
                vyprev=vymsg.data
                vxprev=vxmsg.data
                if(avgcounty<10):
                    avgylist.append(vymsg.data)
                    avgcounty=avgcounty+1
                else:
                    
                    vyavg.data=sum(avgylist)/10.0
                    
                    pub5.publish(vyavg)
                    avgylist.pop(0)
                    avgylist.append(vymsg.data)


                    
	        
                arr.data = [vxavg.data, vyavg.data]
                f1=0
                f2=0
                if(vxavg.data>0 or vxavg.data<0):
                    f1=1
                if(vyavg.data>0 or vyavg.data<0):
                    f2=1
                if(f1==1 and f2==1):       
                    print(arr.data)
                    pub6.publish(arr)
        else:
            currenttime=rospy.Time.now().to_sec()
            prevtime=currenttime           
    else:
        testframe=0
    rate.sleep()            
def main1():
    rospy.init_node('vodom',anonymous =False)
    global rate
    rate=rospy.Rate(20)
    rospy.Subscriber('/ground_truth/state',Odometry,restore,queue_size=1)
    rospy.Subscriber('converted_pc',MarkerArray,callback,queue_size=1)
    rospy.Subscriber('/sensor_imu_hector',Imu,angular,queue_size=1)
    rospy.spin()
if __name__ == '__main__':
    main1()
