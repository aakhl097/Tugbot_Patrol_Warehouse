import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TwistWithCovarianceStamped, Twist
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist, Pose
from std_msgs.msg import Int32, UInt32MultiArray
#from custom_messages.msg import String_Array
import pandas as pd
import csv
import cantools
from std_msgs.msg import Float64

class CAN_Decoder_Node(Node):

    def __init__(self):
        super().__init__('savvycan_csv_decoder')

        self.dbc = cantools.database.load_file(
            'jcrobots_vcu02_canbus_v2.2a.dbc'
        )

        self.csv_file = open('VCU_data.csv', newline='')
        self.reader = csv.DictReader(self.csv_file, delimiter=',')
        self.reader.fieldnames = [h.strip() for h in self.reader.fieldnames]


        self.signal_publishers = {}

        self.timer = self.create_timer(0.02, self.step)

    def parse_can_id(self, raw_id):
        raw_id = raw_id.strip()
        if raw_id.startswith('0x') or raw_id.startswith('0000'):
            return int(raw_id, 16)
        return int(raw_id)

    def step(self):
        try:
            row = next(self.reader)
        except StopIteration:
            self.get_logger().info('CSV finished')
            self.csv_file.close()
            rclpy.shutdown()
            return

        can_id = self.parse_can_id(row['ID'])
        dlc = int(row['LEN'])

        data = bytes(int(row[f'D{i}'], 16) for i in range(1, dlc + 1))

        try:
            msg = self.dbc.get_message_by_frame_id(can_id)
            decoded = msg.decode(data)
        except KeyError:
            return

        for signal, value in decoded.items():
            if signal not in self.signal_publishers: 
                self.signal_publishers[signal] = self.create_publisher(
                    Float64,
                    f'/can/{signal}',
                    10
                )

            ros_msg = Float64()
            # Check if the value is a NamedSignalValue
            if hasattr(value, 'value'):
                ros_msg.data = float(value.value)
            else:
                ros_msg.data = float(value)
            
            self.signal_publishers[signal].publish(ros_msg)
    
"""
    def __init__(self):
        super().__init__('can_decoder_node')

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.list_of_ids = UInt32MultiArray()
        #self.list_of_ids_string_format = UInt32MultiArray()

        # Publishers & Subscribers
        self.arbitration_ids_publisher = self.create_publisher(UInt32MultiArray, '/bhf_delta_robot/can/arbitration_ids', qos)
        self.length_timestamps = 0
        self.length_ids = 0
        self.create_timer(0.004, self.CAN_Message_Decoder)

    def CAN_Message_Decoder(self):

        # 1. Read the data and ensure it doesn't accidentally use a data column as an index
        data = pd.read_csv('VCU_data.csv', sep=',', index_col=False)
        timestamps = data["Time Stamp"].values
        arbitration_ids = data['ID'].values
        self.length_ids = len(arbitration_ids)
        self.length_timestamps = len(timestamps)

        #e.g. ids = [1, 4, 7, 35, 84]

        self.list_of_ids.data = [int (x,16) for x in arbitration_ids]
        self.arbitration_ids_publisher.publish(self.list_of_ids)
        #print("\n")
"""

def main(args=None):
    rclpy.init(args=args)
    node = CAN_Decoder_Node()
    rclpy.spin(node)
    #node.destroy_node()
    #rclpy.shutdown()

if __name__ == '__main__':
    main()

            