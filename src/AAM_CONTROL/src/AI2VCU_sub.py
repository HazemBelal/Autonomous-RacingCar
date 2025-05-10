#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cantools
import can
import threading
import time

# Load the CAN database file
db = cantools.database.load_file("/home/hazem/Downloads/test.dbc")
can_interface = 'vcan0'
bus = can.interface.Bus(can_interface, bustype='socketcan')

# Default values for AI2VCU messages
default_values = {
    'AI2VCU_Brake': {
        'HYD_PRESS_F_REQ_pct': 0,
        'HYD_PRESS_R_REQ_pct': 0
    },
    'AI2VCU_Drive_R': {
        'REAR_AXLE_TRQ_REQUEST': 0,
        'REAR_MOTOR_SPEED_MAX': 0
    },
    'AI2VCU_Steer': {
        'STEER_REQUEST': 0
    },
    'AI2VCU_Drive_F': {
        'FRONT_AXLE_TRQ_REQUEST': 0,
        'FRONT_MOTOR_SPEED_MAX': 0
    },
    'AI2VCU_Status': {
        'DIRECTION_REQUEST': 0,
        'ESTOP_REQUEST': 0,
        'MISSION_STATUS': 0,
        'HANDSHAKE': 0,
        'LAP_COUNTER': 0,
        'CONES_COUNT_ACTUAL': 0,
        'CONES_COUNT_ALL': 0,
        'VEH_SPEED_ACTUAL': 0,
        'VEH_SPEED_DEMAND': 0
    }
}

# List of error signals to monitor
error_signals = [
    'R1_AI2VCU_STATUS_TIMEOUT_ERROR',
    'R1_AI2VCU_DRIVE_F_TIMEOUT_ERROR',
    'R1_AI2VCU_DRIVE_R_TIMEOUT_ERROR',
    'R1_AI2VCU_STATUS_HANDSHAKE_TIMEOUT_ERROR',
    'R1_AI2VCU_STEER_TIMEOUT_ERROR',
    'R1_AI2VCU_BRAKE_TIMEOUT_ERROR',
    'WARN_BRAKE_PLAUSIBILITY',
    'WARN_KL15_UNDER_V',
    'WARN_AI_ESTOP_REQ',
    'WARN_AI_COMMS_LOST',
    'WARN_AUTO_BRAKING',
    'WARN_MISSION_STATUS'
]

class AIVCUPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('/AI2VCU_Status', String, queue_size=10)
        self.error_detected = False
        self.handshake_bit = 0
        rospy.Timer(rospy.Duration(1), self.publish_default_values)

        # Start a separate thread to handle VCU_STATUS messages
        vcu_status_thread = threading.Thread(target=self.handle_vcu_status)
        vcu_status_thread.daemon = True
        vcu_status_thread.start()

    def vcu_status_callback(self, msg):
        rospy.loginfo("Callback called")
        try:
            # Parse the received message data
            data = eval(msg.data.replace("'", '"'))  # Convert single quotes to double quotes for JSON compatibility
            rospy.loginfo(f"Received VCU_STATUS message: {data}")

            # Check if any of the error signals is set to 'TRUE'
            self.error_detected = any(data.get(signal) == 'TRUE' for signal in error_signals)
            rospy.loginfo(f"Error detected: {self.error_detected}")

        except Exception as e:
            rospy.logerr(f"Error processing VCU_STATUS message: {e}")

    def handle_vcu_status(self):
        while not rospy.is_shutdown():
            message = bus.recv(timeout=1.0)
            if message:
                rospy.loginfo(f"Received CAN message with ID {message.arbitration_id}")
            if message and message.arbitration_id == 0x520:  # Listening to VCU2AI_Status
                decoded_message = db.decode_message('VCU2AI_Status', message.data)
                rospy.loginfo(f"Decoded VCU_STATUS message: {decoded_message}")
                vcu_handshake = decoded_message.get('HANDSHAKE', 0)

                # Check VCU handshake response
                if vcu_handshake == 0 or vcu_handshake == 1:
                    self.send_ai2vcu_status(vcu_handshake)
            if message and message.arbitration_id == 0x120:
                decoded_message = db.decode_message('VCU_STATUS', message.data)
                rospy.loginfo(f"Decoded VCU_STATUS message: {decoded_message}")
                ebs_fault_triggered = any(decoded_message.get(signal, 'FALSE') == 'TRUE' for signal in error_signals)
                rospy.loginfo(f"EBS fault triggered: {ebs_fault_triggered}")
                if ebs_fault_triggered:
                    rospy.loginfo("EBS Fault Triggered! Sending updated AI2VCU_Status message.")
                    self.error_detected = True
                else:
                    self.error_detected = False

    def send_ai2vcu_status(self, handshake_value):
        try:
            message_data = default_values['AI2VCU_Status'].copy()
            message_data['HANDSHAKE'] = handshake_value
            if self.error_detected:
                message_data['ESTOP_REQUEST'] = 1  # Set the ESTOP_REQUEST signal to 1 if an error is detected
            else:
                message_data['ESTOP_REQUEST'] = 0  # Reset ESTOP_REQUEST signal if no error is detected

            # Encode the message
            encoded_data = db.encode_message('AI2VCU_Status', message_data)

            # Create and send the CAN message
            can_message = can.Message(arbitration_id=0x510, data=encoded_data, is_extended_id=False)
            bus.send(can_message)
            if handshake_value == self.handshake_bit:
                rospy.loginfo(f"Sent AI2VCU_Status message with correct HANDSHAKE={handshake_value}: {message_data}")
            else:
                rospy.logerr(f"Sent AI2VCU_Status message with incorrect HANDSHAKE={handshake_value}: {message_data}")

            self.handshake_bit = handshake_value

        except Exception as e:
            rospy.logerr(f"Error sending AI2VCU_Status message: {e}")

    def publish_default_values(self, event):
        try:
            # Iterate through each message in default_values and publish it
            for message_name, values in default_values.items():
                message_data = values.copy()
                if message_name == 'AI2VCU_Status' and self.error_detected:
                    message_data['ESTOP_REQUEST'] = 1  # Set the ESTOP_REQUEST signal to 1 if an error is detected

                msg_data = str(message_data)
                rospy.loginfo(f"Publishing {message_name}: {msg_data}")

                # Publish the encoded message as a string
                msg = String()
                msg.data = msg_data
                self.publisher.publish(msg)

                # Send the CAN message
                self.send_can_message(message_name, message_data)

        except Exception as e:
            rospy.logerr(f"Error publishing {message_name} message: {e}")

    def send_can_message(self, message_name, message_data):
        try:
            # Get the CAN message definition from the database
            message = db.get_message_by_name(message_name)
            if message is None:
                raise ValueError(f"Message '{message_name}' not found in the database.")

            # Encode the message
            encoded_data = message.encode(message_data)

            # Create and send the CAN message
            can_message = can.Message(arbitration_id=message.frame_id, data=encoded_data, is_extended_id=False)
            bus.send(can_message)
            rospy.loginfo(f"CAN message sent successfully with data: {message_data}")
        except Exception as e:
            rospy.logerr(f"Error sending CAN message: {e}")

def main():
    rospy.init_node('aivcu_publisher', anonymous=True)
    node = AIVCUPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
