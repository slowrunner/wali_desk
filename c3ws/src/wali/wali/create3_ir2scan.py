# FILE: create3_ir2scan.py

"""

  Purpose:  Subscribe to ir_intensity vector of seven IR readings
            Publish as a LaserScan topic with seven valid ranges per msg

  Limits:  The IR intensity varies with surface reflectivity, surface color, 
           and angle of incidence.  
           The intensity reading to distance function is based on 
             - white painted household baseboard - other surfaces invalidate the function
             - sensor normal to obstacle/wall - other incidence angles invalidate the function
           Reported Distances may be off by 50 to 150mm


  Subscribes: /ir_intensity  irobot_create_msgs/IrIntensityVector.msg

              std_msgs/Header header
              irobot_create_msgs/IrIntensity[] readings

              irobot_create_msg/IrIntensity
                  std_msgs/Header header
                  int16 value

  Publishes: /scan  sensor_msgs/msg/LaserScan.msg
                      std_msgs/Header header
                      float32         angle_min  # start angle of scan (rad)
                      float32         angle_max  # end angle of scan (rad)
                      float32         angle_increment  # distance between measurements (rad)
                      float32         time_increment   # time between measurements [seconds]
                      float32         scan_time        # time between scans [seconds]
                      float32         range_min        # minimum valid range [m]
                      float32         range_max        # maximum valid range [m]
                      float32[]       ranges           # range data [m]
                      float32[]       intensities      # (optional)

  Uses:  

"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import IrIntensityVector

'''
Namespace for robot 
'''
namespace = ''

DEBUG = False
# Uncomment for debug prints to console
DEBUG = True

# pick one of the following qos profile methods - explicit or named profile
# from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data



class IR2Scan_Node(Node):

    def __init__(self):
        super().__init__('ir2scan')
        self.subscription = self.create_subscription(
            IrIntensityVector,
            namespace + '/ir_intensity',
            self.ir_intensity_sub_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def ir_intensity_sub_callback(self, msg:IrIntensityVector):
        self.get_logger().info('ir_intensities: {}'.format(msg))
        self.ir_intensity_vector = msg  # readings[].value


def main(args=None):
    rclpy.init(args=args)

    ir2scan_node = IR2Scan_Node()

    rclpy.spin(ir2scan_node)

    ir2scan_node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
