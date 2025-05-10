#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cantools
import can

# Load the CAN database file
db = cantools.database.load_file("/home/hazem/Downloads/test.dbc")
can_interface = 'vcan0'
bus = can.interface.Bus(can_interface, bustype='socketcan')

class AI2LOGDynamics2Subscriber:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/AI2LOG_Dynamics2', String, self.callback)
        rospy.loginfo("AI2LOG_Dynamics2Subscriber initialized")

    def callback(self, msg):
        try:
            # Parse the received message data
            data = eval(msg.data.replace("'", '"'))  # Convert single quotes to double quotes for JSON compatibility
            rospy.loginfo(f"Received AI2LOG_Dynamics2 message: {data}")

            # Create a dictionary with the message data
            message_data = {
                'Accel_longitudinal_mps2': data['Accel_longitudinal_ms2'],
                'Accel_lateral_mps2': data['Accel_lateral_ms2'],
                'Yaw_rate_degps': data['Yaw_rate_degps']
            }

            # Get the CAN message definition from the database
            message = db.get_message_by_name('AI2LOG_Dynamics2')
            if message is None:
                raise ValueError(f"Message 'AI2LOG_Dynamics2' not found in the database.")

            # Encode the message
            encoded_data = message.encode(message_data)

            # Create and send the CAN message
            can_message = can.Message(arbitration_id=message.frame_id, data=encoded_data, is_extended_id=False)
            bus.send(can_message)
            rospy.loginfo(f"Sent AI2LOG_Dynamics2 message: {message_data}")

        except Exception as e:
            rospy.logerr(f"Error sending AI2LOG_Dynamics2 message: {e}")

def main():
    rospy.init_node('ai2log_dynamics2_subscriber', anonymous=True)
    node = AI2LOGDynamics2Subscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
