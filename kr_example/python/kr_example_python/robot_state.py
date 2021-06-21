import rclpy
from rclpy.node import Node
import sys

from kr_msgs.msg import SystemState


class RobotStateSubscriber(Node):

    def __init__(self):
        super().__init__('robot_state_subscriber')
        self.msgCount = 0
        self.subscription = self.create_subscription(
            SystemState,
            "/kr/system/state",
            self.robot_state_callback,
            10
        )

    def robot_state_callback(self, msg):
        print("  robot_mode              : %s" % msg.robot_mode.str)
        print("  robot_state             : %s" % msg.robot_state.str)
        print("  sensed_pos              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.sensed_pos[0],
            msg.sensed_pos[1],
            msg.sensed_pos[2],
            msg.sensed_pos[3],
            msg.sensed_pos[4],
            msg.sensed_pos[5],
            msg.sensed_pos[6])
              )
        print("  sensed_vel              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.sensed_vel[0],
            msg.sensed_vel[1],
            msg.sensed_vel[2],
            msg.sensed_vel[3],
            msg.sensed_vel[4],
            msg.sensed_vel[5],
            msg.sensed_vel[6])
              )
        print("  sensed_trq              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.sensed_trq[0],
            msg.sensed_trq[1],
            msg.sensed_trq[2],
            msg.sensed_trq[3],
            msg.sensed_trq[4],
            msg.sensed_trq[5],
            msg.sensed_trq[6])
              )
        print("  target_pos              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.target_pos[0],
            msg.target_pos[1],
            msg.target_pos[2],
            msg.target_pos[3],
            msg.target_pos[4],
            msg.target_pos[5],
            msg.target_pos[6])
              )
        print("  target_vel              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.target_vel[0],
            msg.target_vel[1],
            msg.target_vel[2],
            msg.target_vel[3],
            msg.target_vel[4],
            msg.target_vel[5],
            msg.target_vel[6])
              )
        print("  target_trq              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.target_trq[0],
            msg.target_trq[1],
            msg.target_trq[2],
            msg.target_trq[3],
            msg.target_trq[4],
            msg.target_trq[5],
            msg.target_trq[6])
              )
        print("  pos                     : %7.3f %7.3f %7.3f " % (
            msg.pos[0],
            msg.pos[1],
            msg.pos[2])
              )
        print("  rot                     : %7.3f %7.3f %7.3f " % (
            msg.rot[0],
            msg.rot[1],
            msg.rot[2])
              )
        sys.stdout.write("  iob_digital_out         : ")
        for val in msg.iob_digital_out:
            sys.stdout.write("%d " % val)
        print()  # end line
        sys.stdout.write("  iob_relays              : ")
        for val in msg.iob_relays:
            sys.stdout.write("%d " % val)
        print()    # end line
        sys.stdout.write("  iob_current_out         : ")
        for val in msg.iob_current_out:
            sys.stdout.write("%7.3f " % val)
        print()    # end line
        sys.stdout.write("  iob_voltage_out         : ")
        for val in msg.iob_voltage_out:
            sys.stdout.write("%7.3f " % val)
        print()    # end line
        sys.stdout.write("  iob_digital_in          : ")
        for val in msg.iob_digital_in:
            sys.stdout.write("%d " % val)
        print()    # end line
        sys.stdout.write("  iob_safe_in             : ")
        for val in msg.iob_safe_in:
            sys.stdout.write("%d " % val)
        print()    # end line
        sys.stdout.write("  iob_current_in          : ")
        for val in msg.iob_current_in:
            sys.stdout.write("%7.3f " % val)
        print()    # end line
        sys.stdout.write("  iob_voltage_in          : ")
        for val in msg.iob_voltage_in:
            sys.stdout.write("%7.3f " % val)
        print()    # end line
        # TODO quadratures
        sys.stdout.write("  tio_digital_out         : ")
        for val in msg.tio_digital_out:
            sys.stdout.write("%d " % val)
        print()    # end line
        sys.stdout.write("  tio_power_supply        : ")
        for val in msg.tio_power_supply:
            sys.stdout.write("%d " % val)
        print()    # end line
        sys.stdout.write("  tio_analog_out          : ")
        for val in msg.tio_analog_out:
            sys.stdout.write("%7.3f " % val)
        print()    # end line
        sys.stdout.write("  tio_current_in          : ")
        for val in msg.tio_current_in:
            sys.stdout.write("%7.3f " % val)
        print()    # end line
        sys.stdout.write("  tio_voltage_in          : ")
        for val in msg.tio_voltage_in:
            sys.stdout.write("%7.3f " % val)
        print()    # end line


def main(args=None):
    rclpy.init(args=args)

    robot_state_subscriber = RobotStateSubscriber()
    rclpy.spin(robot_state_subscriber)
    robot_state_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
