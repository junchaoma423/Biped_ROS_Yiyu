import numpy as np
import rospy
import time
from unitree_legged_msgs.msg import MotorState, MotorCmd
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import WrenchStamped
from unitree_legged_msgs.msg import LowState, LowCmd, MotorState, MotorCmd, Cartesian

NUM_MOTORS = 12

TOPICS = [
    "/low_cmd",
    "/low_state",
    "/est_base_pos",
    "/est_lin_vel",
]

# INIT_MOTOR_ANGLES = np.array([0, 0.9, -1.8] * 4)
INIT_MOTOR_ANGLES = np.array([0.0, 0.67, -1.3] * 4)


class A1Controller:
    def __init__(self, name="python_controller"):
        self._motor_states = []
        self._motor_commands = []
        self._foot_forces = None
        self._imu = None
        self._base_pos = None
        self._lin_vel = None
        self.count = 0
        self.rec_cnt = 0
        rospy.init_node(name, anonymous=True)
        self._rate = rospy.Rate(1000)

        self._publisher = rospy.Publisher("/low_cmd", LowCmd, queue_size=1)
        rospy.Subscriber("/low_state", LowState, self.low_state_callback)
        rospy.Subscriber("/est_base_pos", Cartesian, self.base_pos_callback)
        rospy.Subscriber("/est_lin_vel", Cartesian, self.lin_vel_callback)

        for i in range(NUM_MOTORS):
            self._motor_states.append(MotorState())
            self._motor_commands.append(MotorCmd())

        time.sleep(1)

    def low_state_callback(self, data):
        self._foot_forces = data.footForce
        self._imu = data.imu
        for i in range(NUM_MOTORS):
            self._motor_states[i] = data.motorState[i]

    def base_pos_callback(self, data):
        self._base_pos = data

    def lin_vel_callback(self, data):
        self._lin_vel = data

    def send_motor_command(self, kpJoint, kdJoint, qDes, qdotDes, tauDes):
        low_cmd = LowCmd()
        low_cmd.levelFlag = 255
        for i in range(NUM_MOTORS):
            cmd = self._motor_commands[i]
            cmd.mode = 0x0A
            cmd.Kp = kpJoint[i]
            cmd.Kd = kdJoint[i]
            cmd.q = qDes[i]
            cmd.dq = qdotDes[i]
            cmd.tau = tauDes[i]
            low_cmd.motorCmd[i] = cmd
        self._publisher.publish(low_cmd)
        self.count += 1
        print("msg sent", self.count)
        self._rate.sleep()
        # time.sleep(0.001)send_motor_command

    def settle_robot(self):
        kp_joint = np.array([60] * 12)
        kd_joint = np.array([5] * 12)
        cur_angles = self.get_motor_angles()
        duration = 2000
        buf_time = 1500
        for i in range(duration):
            percent = i / buf_time
            if percent < 1.0:
                qDes = (1 - percent) * cur_angles + percent * INIT_MOTOR_ANGLES
            else:
                qDes = INIT_MOTOR_ANGLES
            self.send_motor_command(kp_joint, kd_joint, qDes, np.zeros(12), np.zeros(12))

    def get_motor_angles(self):
        q = []
        for motor_state in self._motor_states:
            q.append(motor_state.q)
        return np.array(q)

    def get_motor_velocities(self):
        dq = []
        for motor_state in self._motor_states:
            dq.append(motor_state.dq)
        return np.array(dq)

    def get_base_position(self):
        return np.array([self._base_pos.x, self._base_pos.y, self._base_pos.z])

    def get_base_orientation(self):
        orientation = self._imu.quaternion
        return np.array([orientation[1], orientation[2], orientation[3], orientation[0]])

    def get_base_linear_velocity(self):
        return np.array([self._lin_vel.x, self._lin_vel.y, self._lin_vel.z])

    def get_base_angular_velocity(self):
        angular_vel = self._imu.gyroscope
        return np.array([angular_vel[0], angular_vel[1], angular_vel[2]])

    def get_foot_contacts(self):
        contact_bools = []
        contact_threshold = 10
        for foot_force in self._foot_forces:
            if foot_force >= contact_threshold:
                contact_bools.append(True)
            else:
                contact_bools.append(False)
        return np.array(contact_bools)


# For test purpose
if __name__ == "__main__":
    robot = A1Controller()
    robot.settle_robot()
    rospy.spin()
