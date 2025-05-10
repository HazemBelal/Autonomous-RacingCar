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

# Default values for each AI2VCU message
default_values = {
    'AI2VCU_Brake': {
        'HYD_PRESS_F_REQ_pct': 0,
        'HYD_PRESS_R_REQ_pct': 0
    },
    'AI2VCU_Drive_R': {
        'REAR_AXLE_TRQ_REQUEST': 0,
        'REAR_MOTOR_SPEED_MAX': 0
    },
    'AI2VCU_Status': {
        'HS': 0,    # HANDSHAKE
        'ES': 0,    # ESTOP_REQUEST
        'MS': 0,    # MISSION_STATUS
        'DR': 0,    # DIRECTION_REQUEST
        'LC': 0,    # LAP_COUNTER
        'CCA': 0,   # CONES_COUNT_ACTUAL
        'CCAL': 0,  # CONES_COUNT_ALL
        'VSA': 0,   # VEH_SPEED_ACTUAL
        'VSD': 0    # VEH_SPEED_DEMAND
    },
    'AI2VCU_Steer': {
        'STEER_REQUEST': 0
    },
    'AI2VCU_Drive_F': {
        'FRONT_AXLE_TRQ_REQUEST': 0,
        'FRONT_MOTOR_SPEED_MAX': 0
    },
    'AI2LOG_Dynamics2': {
        'Accel_longitudinal_ms2': 0.0,
        'Accel_lateral_ms2': 0.0,
        'Yaw_rate_degps': 0.0
    }
}

# Default values for VCU_STATUS message
vcu_status_values = {
    'SM_SYS': 0,
    'SM_AS': 0,
    'SYS_ACTION_STATE': 1,
    'R1_AI2VCU_STATUS_TIMEOUT_ERROR': 0,
    'R1_AI2VCU_DRIVE_F_TIMEOUT_ERROR': 0,
    'R1_AI2VCU_DRIVE_R_TIMEOUT_ERROR': 0,
    'R1_AI2VCU_STEER_TIMEOUT_ERROR': 0,
    'R1_AI2VCU_BRAKE_TIMEOUT_ERROR': 0,
    'R1_AI2VCU_STATUS_HANDSHAKE_TIMEOUT_ERROR': 1,
    'WARN_AI_ESTOP_REQ': 0,
    'WARN_AUTO_BRAKING': 0,
    'WARN_MISSION_STATUS': 0,
    'WARN_KL15_UNDER_V': 0,
    'WARN_BRAKE_PLAUSIBILITY': 0,
    'WARN_AI_COMMS_LOST': 0,
}

# Map the longer version of signal names to the smaller version for AI2VCU_Status
status_signal_mapping = {
    'HANDSHAKE': 'HS',
    'ESTOP_REQUEST': 'ES',
    'MISSION_STATUS': 'MS',
    'DIRECTION_REQUEST': 'DR',
    'LAP_COUNTER': 'LC',
    'CONES_COUNT_ACTUAL': 'CCA',
    'CONES_COUNT_ALL': 'CCAL',
    'VEH_SPEED_ACTUAL': 'VSA',
    'VEH_SPEED_DEMAND': 'VSD'
}

class AIVCUPublisher:
    publishers = {}

    def __init__(self):
        self.create_publishers()

    def create_publishers(self):
        for message_name in default_values.keys():
            self.publishers[message_name] = rospy.Publisher(f'/{message_name}', String, queue_size=10)

    def publish_default_values(self):
        while not rospy.is_shutdown():
            for message_name, values in default_values.items():
                # Encode the message using the default values
                msg_data = self.format_message_data(message_name, values)
                rospy.loginfo(f"Publishing default values for {message_name}: {msg_data}")

                # Publish the encoded message as a string
                msg = String()
                msg.data = msg_data
                self.publishers[message_name].publish(msg)

            # Publish VCU_STATUS message
            self.publish_vcu_status()

            # Delay between iterations
            time.sleep(1)  # 1 second delay

    def publish_vcu_status(self):
        try:
            # Get the CAN message definition from the database
            message = db.get_message_by_name('VCU_STATUS')
            if message is None:
                raise ValueError(f"Message 'VCU_STATUS' not found in the database.")

            # Encode the message
            encoded_data = message.encode(vcu_status_values)

            # Create a CAN message
            can_message = can.Message(arbitration_id=message.frame_id, data=encoded_data, is_extended_id=False)

            # Send the message over the virtual CAN interface
            bus.send(can_message)
            rospy.loginfo(f"VCU_STATUS message sent successfully with data: {vcu_status_values}")
        except Exception as e:
            rospy.logerr(f"Error sending VCU_STATUS message: {e}")

    def format_message_data(self, message_name, values):
        if message_name == 'AI2VCU_Status':
            formatted_values = {}
            for signal_name, value in values.items():
                if signal_name in status_signal_mapping:
                    formatted_values[status_signal_mapping[signal_name]] = value
                else:
                    formatted_values[signal_name] = value
            return str(formatted_values)
        else:
            return str(values)

def main():
    rospy.init_node('aivcu_publisher', anonymous=True)
    node = AIVCUPublisher()
    thread = threading.Thread(target=node.publish_default_values)
    thread.daemon = True
    thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()
