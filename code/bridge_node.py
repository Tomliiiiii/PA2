#!/usr/bin/env python3
"""
RaspRover Bridge Node
=====================
Connects ROS2 to the ESP32 sub-controller of the Waveshare RaspRover (UART, JSON).

Subscribes:
  /cmd_vel          geometry_msgs/Twist   -> sent continuously as {"T":13,"X":..,"Z":..}

Publishes:
  /odom             nav_msgs/Odometry     (distance from odl/odr, heading from gyro gz)
  /imu/data_raw     sensor_msgs/Imu       (acceleration + angular rate, no orientation)
  /battery_state    sensor_msgs/BatteryState
  TF odom -> base_link (optional)

Conversions (measured on the robot, Oct. 2026):
  odl/odr : cm (integer, cumulative, signed)
  L/R     : m/s
  ax..az  : raw / 8192 = g
  gx..gz  : (raw - offset) / 16.4 = deg/s
  v       : raw / 100 = V
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from tf2_ros import TransformBroadcaster

G = 9.80665


class RaspRoverBridge(Node):

    def __init__(self):
        super().__init__('rasprover_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_rate', 20.0)          # Hz, command rate to the ESP32
        self.declare_parameter('cmd_timeout', 0.5)        # s, stop after this time without /cmd_vel
        self.declare_parameter('odom_scale', 0.01)        # m per odl/odr unit
        self.declare_parameter('accel_scale', 8192.0)     # raw value per g
        self.declare_parameter('gyro_scale', 16.4)        # raw value per deg/s
        self.declare_parameter('gyro_calib_time', 2.0)    # s at standstill for the offset
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        p = lambda n: self.get_parameter(n).value
        self.odom_scale = p('odom_scale')
        self.accel_scale = p('accel_scale')
        self.gyro_scale = p('gyro_scale')
        self.gyro_calib_time = p('gyro_calib_time')
        self.cmd_timeout = p('cmd_timeout')
        self.publish_tf = p('publish_tf')
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.imu_frame = p('imu_frame')

        # ---------- Serial connection ----------
        self.ser = serial.Serial(p('port'), p('baud'), timeout=0.1)
        self.write_lock = threading.Lock()
        self.send({"T": 900, "main": 1, "module": 0})   # chassis type RaspRover
        time.sleep(0.1)
        self.send({"T": 131, "cmd": 1})                 # continuous feedback on
        self.get_logger().info(f"Serial port {p('port')} opened")

        # ---------- State ----------
        self.cmd_x = 0.0
        self.cmd_z = 0.0
        self.last_cmd_time = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_odl = None
        self.prev_odr = None
        self.prev_stamp = None

        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calib_samples = []
        self.calib_start = None
        self.calibrated = False

        # ---------- ROS interfaces ----------
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.bat_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / p('cmd_rate'), self.send_cmd)

        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

        self.get_logger().info(
            f"Gyro calibration: keep the robot still for {self.gyro_calib_time:.0f} s ...")

    # ================= Sending =================
    def send(self, cmd: dict):
        line = json.dumps(cmd, separators=(',', ':')) + '\n'
        with self.write_lock:
            self.ser.write(line.encode('utf-8'))

    def on_cmd_vel(self, msg: Twist):
        self.cmd_x = msg.linear.x
        self.cmd_z = msg.angular.z
        self.last_cmd_time = time.time()

    def send_cmd(self):
        # Stand still during calibration and after a command timeout
        if not self.calibrated or time.time() - self.last_cmd_time > self.cmd_timeout:
            x, z = 0.0, 0.0
        else:
            x, z = self.cmd_x, self.cmd_z
        self.send({"T": 13, "X": round(x, 3), "Z": round(z, 3)})

    # ================= Receiving =================
    def read_loop(self):
        buf = b''
        while self.running and rclpy.ok():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                try:
                    data = json.loads(line.decode('utf-8', errors='ignore'))
                except json.JSONDecodeError:
                    continue
                if data.get('T') == 1001:
                    self.handle_feedback(data)

    def handle_feedback(self, d: dict):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        # ----- Determine the gyro offset at start-up -----
        if not self.calibrated:
            if self.calib_start is None:
                self.calib_start = t
            self.calib_samples.append((d['gx'], d['gy'], d['gz']))
            if t - self.calib_start >= self.gyro_calib_time and len(self.calib_samples) > 10:
                n = len(self.calib_samples)
                self.gyro_bias = [sum(s[i] for s in self.calib_samples) / n for i in range(3)]
                self.calibrated = True
                self.get_logger().info(
                    f"Gyro offset determined ({n} samples): "
                    f"gx={self.gyro_bias[0]:.1f} gy={self.gyro_bias[1]:.1f} gz={self.gyro_bias[2]:.1f}")
            self.prev_odl, self.prev_odr, self.prev_stamp = d['odl'], d['odr'], t
            return

        # ----- Unit conversion -----
        deg2rad = math.pi / 180.0
        ax = d['ax'] / self.accel_scale * G
        ay = d['ay'] / self.accel_scale * G
        az = d['az'] / self.accel_scale * G
        gx = (d['gx'] - self.gyro_bias[0]) / self.gyro_scale * deg2rad
        gy = (d['gy'] - self.gyro_bias[1]) / self.gyro_scale * deg2rad
        gz = (d['gz'] - self.gyro_bias[2]) / self.gyro_scale * deg2rad
        v_left, v_right = float(d['L']), float(d['R'])
        voltage = d['v'] / 100.0

        # ----- Odometry -----
        dt = t - self.prev_stamp
        dl = (d['odl'] - self.prev_odl) * self.odom_scale
        dr = (d['odr'] - self.prev_odr) * self.odom_scale
        # Ignore jumps (e.g. counter reset after an ESP32 restart)
        if abs(dl) > 0.5 or abs(dr) > 0.5:
            self.get_logger().warn("Odometry jump detected, step ignored")
            dl = dr = 0.0
        self.prev_odl, self.prev_odr, self.prev_stamp = d['odl'], d['odr'], t

        if 0.0 < dt < 1.0:
            # Heading from the gyro (skid steer slips when turning)
            yaw_mid = self.yaw + gz * dt / 2.0
            self.yaw += gz * dt
            dist = (dl + dr) / 2.0
            self.x += dist * math.cos(yaw_mid)
            self.y += dist * math.sin(yaw_mid)
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        stamp = now.to_msg()

        # ----- /odom -----
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = (v_left + v_right) / 2.0
        odom.twist.twist.angular.z = gz
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        # ----- TF odom -> base_link -----
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        # ----- /imu/data_raw -----
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation_covariance[0] = -1.0          # no orientation provided
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        for i in (0, 4, 8):
            imu.angular_velocity_covariance[i] = 0.0004
            imu.linear_acceleration_covariance[i] = 0.01
        self.imu_pub.publish(imu)

        # ----- /battery_state -----
        bat = BatteryState()
        bat.header.stamp = stamp
        bat.voltage = voltage
        bat.present = True
        bat.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        # rough estimate for 3S: 9.6 V = empty, 12.6 V = full
        bat.percentage = max(0.0, min(1.0, (voltage - 9.6) / 3.0))
        self.bat_pub.publish(bat)

    # ================= Shutdown =================
    def destroy_node(self):
        self.running = False
        try:
            self.send({"T": 13, "X": 0, "Z": 0})
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RaspRoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
