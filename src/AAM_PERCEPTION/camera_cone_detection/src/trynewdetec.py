#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from torchvision.transforms import functional as F  # Add this import for torchvision transforms
from std_msgs.msg import String

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.model = torch.hub.load('/home/andrew/aamfsd_2024_sim/src/AAM_PERCEPTION/yolov5_ros/src/yolov5', 'custom', path='/home/andrew/aamfsd_2024_sim/src/AAM_PERCEPTION/yolov5_ros/weights/small.pt',source='local').to(self.device)
        self.image_pub = rospy.Publisher('image_topic', Image, queue_size=1)
        rospy.Subscriber('/zed/left/image_rect_color', Image, self.image_callback)

    def image_callback(self, msg):
        rospy.loginfo('Received image')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return

        results = self.showimgandgetres(cv_image)

    def showimgandgetres(self, cv_image):
        image_tensor = F.to_tensor(cv_image).unsqueeze(0).to(self.device)
        results = self.model(cv_image)
        cv2_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        # Plot the image with bounding boxes
        for box in results.xyxy[0]:
            # Extract box coordinates and confidence score
            x1, y1, x2, y2, conf, cls = box.tolist()

            # Convert confidence score to percentage
            conf_percent = f'{conf * 100:.2f}%'

            # Draw bounding box
            cv2.rectangle(cv2_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            # Add label and confidence score
            label = f'{self.model.names[int(cls)]}: {conf_percent}'
            color=(0,0,0)
            if self.model.names[int(cls)]=="blue_cone":
                color=(0,0,255)
            elif self.model.names[int(cls)]=="yellow_cone":
                color=(255,255,0)
            elif self.model.names[int(cls)]=="big_orange_cone":
                color=(255,165,0)
                        
            cv2.putText(cv2_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('Annotated Image', cv2_image)
        cv2.waitKey(1)
        ros_image = self.bridge.cv2_to_imgmsg(cv2_image, encoding='bgr8')
        self.image_pub.publish(ros_image)
        return results

def main():
    image_subscriber = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()