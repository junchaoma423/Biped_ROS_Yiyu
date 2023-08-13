import numpy as np
import rospy
import time
from unitree_legged_msgs.msg import MotorState, MotorCmd
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import WrenchStamped

MOTOR_TOPICS = [
    "/a1_gazebo/FR_hip_controller/",
    "/a1_gazebo/FR_thigh_controller/",
    "/a1_gazebo/FR_calf_controller/",

    "/a1_gazebo/FL_hip_controller/",
    "/a1_gazebo/FL_thigh_controller/",
    "/a1_gazebo/FL_calf_controller/",

    "/a1_gazebo/RR_hip_controller/",
    "/a1_gazebo/RR_thigh_controller/",
    "/a1_gazebo/RR_calf_controller/",

    "/a1_gazebo/RL_hip_controller/",
    "/a1_gazebo/RL_thigh_controller/",
    "/a1_gazebo/RL_calf_controller/",
]
MODEL_TOPIC = "/gazebo/model_states"
CONTACT_TOPICS = [
    "/visual/a1_gazebo/FR_foot_contact/the_force",
    "/visual/a1_gazebo/FL_foot_contact/the_force",
    "/visual/a1_gazebo/RR_foot_contact/the_force",
    "/visual/a1_gazebo/RL_foot_contact/the_force",
]

# INIT_MOTOR_ANGLES = np.array([0, 0.9, -1.8] * 4)
INIT_MOTOR_ANGLES = np.array([0.0, 0.67, -1.3] * 4)


class A1Controller:
    def __init__(self, name="python_controller"):
        self._publishers = []
        self._motor_states = []
        self._model_state = None
        self._contact_states = []
        self._motor_commands = []

        rospy.init_node(name, anonymous=True)
        self._rate = rospy.Rate(1000)

        for i in range(len(MOTOR_TOPICS)):
            self._motor_states.append(MotorState())
            rospy.Subscriber(MOTOR_TOPICS[i] + "state", MotorState, self.motor_subscriber_callback, callback_args=i)
            self._publishers.append(rospy.Publisher(MOTOR_TOPICS[i] + "command", MotorCmd, queue_size=1))
            self._motor_commands.append(MotorCmd())

        rospy.Subscriber(MODEL_TOPIC, ModelStates, self.model_subscriber_callback)
        for i in range(len(CONTACT_TOPICS)):
            self._contact_states.append(WrenchStamped())
            rospy.Subscriber(CONTACT_TOPICS[i], WrenchStamped, self.contact_subscriber_callback, callback_args=i)
        time.sleep(1)

    def motor_subscriber_callback(self, data, idx):
        self._motor_states[idx] = data

    def model_subscriber_callback(self, data):
        self._model_state = data

    def contact_subscriber_callback(self, data, idx):
        self._contact_states[idx] = data

    def send_motor_command(self, kpJoint, kdJoint, qDes, qdotDes, tauDes):
        for i in range(len(MOTOR_TOPICS)):
            cmd = self._motor_commands[i]
            cmd.mode = 0x0A
            cmd.Kp = kpJoint[i]
            cmd.Kd = kdJoint[i]
            cmd.q = qDes[i]
            cmd.dq = qdotDes[i]
            cmd.tau = tauDes[i]
            self._publishers[i].publish(cmd)

        self._rate.sleep()

    def settle_robot(self):
        kp_joint = np.array([60] * 12)
        kd_joint = np.array([5] * 12)
        cur_angles = self.get_motor_angles()
        duration = 1000
        buf_time = 800
        for i in range(duration):
            percent = min(1.0, i / buf_time)
            qDes = (1 - percent) * cur_angles + percent * INIT_MOTOR_ANGLES
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
        position = self._model_state.pose[2].position
        print(position)
        return np.array([position.x, position.y, position.z])

    def get_base_orientation(self):
        orientation = self._model_state.pose[2].orientation
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    def get_base_linear_velocity(self):
        linear_vel = self._model_state.twist[2].linear
        return np.array([linear_vel.x, linear_vel.y, linear_vel.z])

    def get_base_angular_velocity(self):
        angular_vel = self._model_state.twist[2].angular
        return np.array([angular_vel.x, angular_vel.y, angular_vel.z])

    def get_foot_contacts(self):
        contact_bools = []
        contact_threshold = 15
        for contact_state in self._contact_states:
            force_z = contact_state.wrench.force.z
            if force_z >= contact_threshold:
                contact_bools.append(True)
            else:
                contact_bools.append(False)
        return np.array(contact_bools)


# For test purpose
if __name__ == "__main__":
    robot = A1Controller()
    robot.settle_robot()
