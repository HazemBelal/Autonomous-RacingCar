#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from eufs_msgs.msg import WheelSpeedsStamped, CanState
import cantools
import can
import threading
from time import time

# Load the CAN database file
db = cantools.database.load_file("/home/hazem/Downloads/test.dbc")

# Create a virtual CAN interface
can_interface = 'vcan0'
bus = can.interface.Bus(can_interface, bustype='socketcan')

# Default values for VCU2AI_Status message
default_values_vcu2ai = {
    'HANDSHAKE': 0,
    'SHUTDOWN_REQUEST': 0,
    'AS_SWITCH_STATUS': 0,
    'TS_SWITCH_STATUS': 0,
    'GO_SIGNAL': 0,
    'STEERING_STATUS': 0,
    'AS_STATE': 1,  # AS_OFF by default
    'AMI_STATE': 0,
    'FAULT_STATUS': 0,
    'WARNING_STATUS': 0,
    'WARN_BATT_TEMP_HIGH': 0,
    'WARN_BATT_SOC_LOW': 0,
    'AI_ESTOP_REQUEST': 0,
    'HVIL_OPEN_FAULT': 0,
    'HVIL_SHORT_FAULT': 0,
    'EBS_FAULT': 0,  # Initially, EBS_FAULT is set to 0
    'OFFBOARD_CHARGER_FAULT': 0,
    'AI_COMMS_LOST': 0,
    'AUTONOMOUS_BRAKING_FAULT': 0,
    'MISSION_STATUS_FAULT': 0,
    'CHARGE_PROCEDURE_FAULT': 0,
    'BMS_FAULT': 0,
    'BRAKE_PLAUSIBILITY_FAULT': 0,
    'SHUTDOWN_CAUSE': 0
}

class VCU2AISubscriber:
    def __init__(self):
        self.handshake_bit = 0  # Initial handshake bit
        self.last_handshake_time = time()
        self.ai_comms_lost = False
        self.publisher = rospy.Publisher('/VCU_STATUS', String, queue_size=10)
        rospy.Subscriber('/ros_can/wheel_speeds', WheelSpeedsStamped, self.ws_callback)
        rospy.Subscriber('/ros_can/state', CanState, self.state_callback)
        
        # Start a separate thread to handle AI2VCU_Status messages
        ai2vcu_status_thread = threading.Thread(target=self.handle_ai2vcu_status)
        ai2vcu_status_thread.daemon = True
        ai2vcu_status_thread.start()

    def handle_ai2vcu_status(self):
        while not rospy.is_shutdown():
            message = bus.recv(timeout=0.05)
            current_time = time()
            if message:
                rospy.loginfo(f"Received CAN message with ID {message.arbitration_id}")
            if message and message.arbitration_id == 0x510:  # Listening to AI2VCU_Status
                decoded_message = db.decode_message('AI2VCU_Status', message.data)
                ai_handshake = decoded_message.get('HANDSHAKE', 0)

                # Check AI handshake response and toggle the handshake bit
                if ai_handshake == self.handshake_bit:
                    rospy.loginfo("CORRECT HANDSHAKE")
                    self.handshake_bit = 1 - self.handshake_bit  # Toggle handshake bit
                    self.last_handshake_time = current_time  # Update last handshake time
                else:
                    rospy.loginfo(f"Expected handshake {self.handshake_bit}, but got {ai_handshake}")

                self.send_vcu2ai_status()
            elif current_time - self.last_handshake_time > 0.1 and not self.ai_comms_lost:
                self.trigger_ai_comms_lost()
                self.ai_comms_lost = True
            elif current_time - self.last_handshake_time > 0.05 and not self.ai_comms_lost:
                self.trigger_ebs_fault()

    def send_vcu2ai_status(self):
        if self.ai_comms_lost:
            return
        try:
            message_data = default_values_vcu2ai.copy()
            message_data['HANDSHAKE'] = self.handshake_bit
            
            # Encode the message
            encoded_data = db.encode_message('VCU2AI_Status', message_data)
            
            # Create and send the CAN message
            can_message = can.Message(arbitration_id=0x520, data=encoded_data, is_extended_id=False)
            bus.send(can_message)
            rospy.loginfo(f"Sent VCU2AI_Status message with HANDSHAKE={self.handshake_bit}: {message_data}")

        except Exception as e:
            rospy.logerr(f"Error sending VCU2AI_Status message: {e}")

    def trigger_ebs_fault(self):
        try:
            message_data = default_values_vcu2ai.copy()
            message_data['EBS_FAULT'] = 1  # Set the EBS_FAULT signal to 1

            # Encode the message
            encoded_data = db.encode_message('VCU2AI_Status', message_data)

            # Create and send the CAN message
            can_message = can.Message(arbitration_id=0x520, data=encoded_data, is_extended_id=False)
            bus.send(can_message)
            rospy.logerr(f"EBS_FAULT triggered! Sent VCU2AI_Status message: {message_data}")
        except Exception as e:
            rospy.logerr(f"Error triggering EBS_FAULT: {e}")

    def trigger_ai_comms_lost(self):
        try:
            message_data = default_values_vcu2ai.copy()
            message_data['AI_COMMS_LOST'] = 1  # Set the AI_COMMS_LOST signal to 1

            # Encode the message
            encoded_data = db.encode_message('VCU2AI_Status', message_data)

            # Create and send the CAN message
            can_message = can.Message(arbitration_id=0x520, data=encoded_data, is_extended_id=False)
            bus.send(can_message)
            rospy.logerr(f"AI_COMMS_LOST triggered! Sent VCU2AI_Status message: {message_data}")
        except Exception as e:
            rospy.logerr(f"Error triggering AI_COMMS_LOST: {e}")

    def ws_callback(self, msg):
        FL_WHEEL_SPEED = msg.speeds.lf_speed
        FR_WHEEL_SPEED = msg.speeds.rf_speed
        RL_WHEEL_SPEED = msg.speeds.lb_speed
        RR_WHEEL_SPEED = msg.speeds.rb_speed

        # Encode the message
        message_data = db.encode_message('VCU2AI_Speeds', {
            'FL_WHEEL_SPEED': FL_WHEEL_SPEED,
            'FR_WHEEL_SPEED': FR_WHEEL_SPEED,
            'RL_WHEEL_SPEED': RL_WHEEL_SPEED,
            'RR_WHEEL_SPEED': RR_WHEEL_SPEED,
        })

        # Create a CAN message
        can_message = can.Message(arbitration_id=0x525, data=message_data)

        # Send the message over the virtual CAN interface
        bus.send(can_message)

    def state_callback(self, msg):
        as_state = msg.as_state
        ami_state = msg.ami_state

        as_state_mapping = {
            0: 'AS_OFF',
            1: 'AS_READY',
            2: 'AS_DRIVING',
            3: 'AS_EMERGENCY_BRAKE',
            4: 'AS_FINISHED'
        }
        ami_state_mapping = {
            10: 'AMI_NOT_SELECTED',
            11: 'AMI_ACCELERATION',
            12: 'AMI_SKIDPAD',
            13: 'AMI_AUTOCROSS',
            14: 'AMI_TRACK_DRIVE',
            15: 'AMI_AUTONOMOUS_DEMO',
            16: 'AMI_ADS_INSPECTION',
            17: 'AMI_ADS_EBS',
            18: 'AMI_DDT_INSPECTION_A',
            19: 'AMI_DDT_INSPECTION_B',
            20: 'AMI_JOYSTICK',
            21: 'AMI_MANUAL'
        }

        mapped_as_state = as_state_mapping.get(as_state, 'AS_OFF')
        mapped_ami_state = ami_state_mapping.get(ami_state, 'AMI_NOT_SELECTED')

        mapped_as_state_db = {
            'AS_OFF': 1,
            'AS_READY': 2,
            'AS_DRIVING': 3,
            'AS_EMERGENCY_BRAKE': 4,
            'AS_FINISHED': 5
        }.get(mapped_as_state, 1)
        mapped_ami_state_db = {
            'AMI_NOT_SELECTED': 0,
            'AMI_ACCELERATION': 1,
            'AMI_SKIDPAD': 2,
            'AMI_AUTOCROSS': 3,
            'AMI_TRACK_DRIVE': 4,
            'AMI_AUTONOMOUS_DEMO': 7,
            'AMI_DDT_INSPECTION_A': 5,
            'AMI_DDT_INSPECTION_B': 6,
            'AMI_JOYSTICK': 10,
            'AMI_MANUAL': 11
        }.get(mapped_ami_state, 0)

        message_data = db.encode_message('VCU2AI_Status', {
            'HANDSHAKE': default_values_vcu2ai['HANDSHAKE'],
            'SHUTDOWN_REQUEST': default_values_vcu2ai['SHUTDOWN_REQUEST'],
            'AS_SWITCH_STATUS': default_values_vcu2ai['AS_SWITCH_STATUS'],
            'TS_SWITCH_STATUS': default_values_vcu2ai['TS_SWITCH_STATUS'],
            'GO_SIGNAL': default_values_vcu2ai['GO_SIGNAL'],
            'STEERING_STATUS': default_values_vcu2ai['STEERING_STATUS'],
            'AS_STATE': mapped_as_state_db,
            'AMI_STATE': mapped_ami_state_db,
            'FAULT_STATUS': default_values_vcu2ai['FAULT_STATUS'],
            'WARNING_STATUS': default_values_vcu2ai['WARNING_STATUS'],
            'WARN_BATT_TEMP_HIGH': default_values_vcu2ai['WARN_BATT_TEMP_HIGH'],
            'WARN_BATT_SOC_LOW': default_values_vcu2ai['WARN_BATT_SOC_LOW'],
            'AI_ESTOP_REQUEST': default_values_vcu2ai['AI_ESTOP_REQUEST'],
            'HVIL_OPEN_FAULT': default_values_vcu2ai['HVIL_OPEN_FAULT'],
            'HVIL_SHORT_FAULT': default_values_vcu2ai['HVIL_SHORT_FAULT'],
            'EBS_FAULT': default_values_vcu2ai['EBS_FAULT'],
            'OFFBOARD_CHARGER_FAULT': default_values_vcu2ai['OFFBOARD_CHARGER_FAULT'],
            'AI_COMMS_LOST': default_values_vcu2ai['AI_COMMS_LOST'],
            'AUTONOMOUS_BRAKING_FAULT': default_values_vcu2ai['AUTONOMOUS_BRAKING_FAULT'],
            'MISSION_STATUS_FAULT': default_values_vcu2ai['MISSION_STATUS_FAULT'],
            'CHARGE_PROCEDURE_FAULT': default_values_vcu2ai['CHARGE_PROCEDURE_FAULT'],
            'BMS_FAULT': default_values_vcu2ai['BMS_FAULT'],
            'BRAKE_PLAUSIBILITY_FAULT': default_values_vcu2ai['BRAKE_PLAUSIBILITY_FAULT'],
            'SHUTDOWN_CAUSE': default_values_vcu2ai['SHUTDOWN_CAUSE']
        })

        can_message = can.Message(arbitration_id=0x520, data=message_data)
        bus.send(can_message)

def main():
    rospy.init_node('vcu2ai_subscriber', anonymous=True)
    node = VCU2AISubscriber()
    
    rospy.spin()
    bus.shutdown()

if __name__ == '__main__':
    main()
