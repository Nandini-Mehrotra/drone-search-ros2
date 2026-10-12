
#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class DroneSearchController(Node):
    def __init__(self, fly=False):
        super().__init__("drone_search_controller")

        self.fly = fly
        self.state = "WAITING"
        self.position = None
        self.status = None
        self.home = None
        self.start_time = None
        self.last_log = 0.0
        self.offboard_start = None
        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.land_requested = False

        # Simulation configuration
        self.takeoff_height = 5.0
        self.search_radius = 6.0
        self.max_radius = 12.0
        self.max_height = 8.0
        self.search_duration = 45.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos,
        )

        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            qos,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        self.create_timer(0.1, self.control_loop)

        mode = "FLIGHT ENABLED" if fly else "DRY RUN"
        self.get_logger().info(f"Controller started: {mode}")

    def timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def position_callback(self, msg):
        self.position = msg

        if self.home is None and msg.xy_valid and msg.z_valid:
            self.home = (msg.x, msg.y, msg.z)
            self.get_logger().info(
                f"Home recorded: {self.home}"
            )

    def status_callback(self, msg):
        self.status = msg

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.yaw = math.nan
        self.trajectory_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(
                f"STATE: {self.state} -> {new_state}"
            )
            self.state = new_state

    def geofence_violated(self):
        if self.position is None or self.home is None:
            return False

        dx = self.position.x - self.home[0]
        dy = self.position.y - self.home[1]
        height = self.home[2] - self.position.z

        radius = math.hypot(dx, dy)

        return radius > self.max_radius or height > self.max_height

    def request_landing(self):
        if self.land_requested:
            return

        self.land_requested = True
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().warn("Landing requested")

    def control_loop(self):
        now = time.monotonic()

        if self.position is None or self.status is None:
            if now - self.last_log > 3:
                self.get_logger().info("Waiting for PX4 telemetry...")
                self.last_log = now
            return

        if self.home is None:
            return

        if not self.position.xy_valid or not self.position.z_valid:
            if self.fly and self.state in ("TAKEOFF", "SEARCH"):
                self.set_state("FAILSAFE")
                self.request_landing()
            return

        # Safety boundary checks use measured position, not setpoints.
        if self.geofence_violated() and self.state not in (
            "FAILSAFE", "LANDING", "DONE"
        ):
            self.get_logger().error("GEOFENCE VIOLATION")
            self.set_state("FAILSAFE")
            if self.fly:
                self.request_landing()
            return

        if self.state == "WAITING":
            if not self.fly:
                if now - self.last_log > 3:
                    self.get_logger().info(
                        "DRY RUN: telemetry OK; flight commands disabled"
                    )
                    self.last_log = now
                return

            if not self.status.pre_flight_checks_pass:
                if now - self.last_log > 3:
                    self.get_logger().warn(
                        "Waiting: PX4 preflight checks have not passed"
                    )
                    self.last_log = now
                return

            self.offboard_start = now
            self.set_state("PREPARE")

        elif self.state == "PREPARE":
            self.publish_offboard_heartbeat()
            self.publish_setpoint(
                self.home[0], self.home[1], self.home[2]
            )

            # PX4 requires setpoints before entering Offboard mode.
            if now - self.offboard_start > 2.0:
                if now - self.last_mode_request > 2.0:
                    self.send_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                        1, 6
                    )
                    self.last_mode_request = now

                if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    if now - self.last_arm_request > 2.0:
                        self.send_command(
                            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            1
                        )
                        self.last_arm_request = now

                    if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                        self.set_state("TAKEOFF")

        elif self.state == "TAKEOFF":
            self.publish_offboard_heartbeat()
            target_z = self.home[2] - self.takeoff_height

            self.publish_setpoint(
                self.home[0], self.home[1], target_z
            )

            if abs(self.position.z - target_z) < 0.5:
                self.start_time = now
                self.set_state("SEARCH")

        elif self.state == "SEARCH":
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.set_state("FAILSAFE")
                self.request_landing()
                return

            elapsed = now - self.start_time

            if elapsed >= self.search_duration:
                self.set_state("LANDING")
                self.request_landing()
                return

            # Figure-8 trajectory in the local NED coordinate frame.
            omega = 2 * math.pi / 20.0
            x = self.home[0] + self.search_radius * math.sin(omega * elapsed)
            y = self.home[1] + 0.5 * self.search_radius * math.sin(
                2 * omega * elapsed
            )
            z = self.home[2] - self.takeoff_height

            self.publish_offboard_heartbeat()
            self.publish_setpoint(x, y, z)

        elif self.state in ("LANDING", "FAILSAFE"):
            # PX4's autonomous landing mode owns the landing trajectory.
            if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.set_state("DONE")

        if now - self.last_log > 5:
            self.get_logger().info(
                f"State={self.state}, "
                f"position=({self.position.x:.1f}, "
                f"{self.position.y:.1f}, "
                f"{self.position.z:.1f})"
            )
            self.last_log = now


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fly",
        action="store_true",
        help="Enable flight commands (simulation only)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = DroneSearchController(fly=args.fly)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stopping an armed offboard controller can trigger PX4's
        # configured offboard-loss failsafe. Test only in simulation.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
