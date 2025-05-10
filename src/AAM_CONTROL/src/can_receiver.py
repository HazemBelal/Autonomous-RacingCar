import rospy
import cantools
import can

# Load the CAN database file
dbc = cantools.database.load_file("/home/hazem/Downloads/test.dbc")

class CANReceiver:
    def __init__(self):
        # Initialize CAN bus
        self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

    def receive_can_message(self):
        while not rospy.is_shutdown():
            message = self.bus.recv(timeout=1.0)
            if message is not None:
                rospy.loginfo(f"Received CAN message: {message}")
                self.parse_can_message(message)

    def parse_can_message(self, message):
        try:
            message_object = dbc.get_message_by_frame_id(message.arbitration_id)
            decoded_message = dbc.decode_message(message.arbitration_id, message.data)
            sensor_name = message_object.name
            for signal, value in decoded_message.items():
                rospy.loginfo(f"Publishing {signal} for {sensor_name}: {value}")
        except KeyError:
            pass  # Ignore messages not found in the database

    def close(self):
        # Close the CAN bus interface
        self.bus.shutdown()

    def __del__(self):
        self.close()

def main():
    rospy.init_node('can_receiver', anonymous=True)
    receiver = CANReceiver()
    receiver.receive_can_message()

if __name__ == '__main__':
    main()
