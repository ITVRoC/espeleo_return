#!/usr/bin/env python
# encoding: utf-8

import rospy
import time
import threading
from math import sqrt
from geometry_msgs.msg import Polygon, Point
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from espeleo_control.msg import Path
from tkinter import Tk, LEFT
from tkinter.ttk import Button, Label, Style

# This script is responsible to record the path traveled by the robot in an array/buffer and also for
# sending this array to the script vec_field_control.py located in espeleo_control package, when the radio connection is lost.

# Parameters
delta = rospy.get_param("d_displacement_limit")  # Minimum variance in the position of the robot to record the new position.
tolerance = rospy.get_param("d_tolerance")  # Tolerance between the final point of stop set by the array and the actual position of the robot.
size = rospy.get_param("d_return")  # Maximum distance to return, stored in the array in meters.
intermediate_dist = rospy.get_param("d_intermediate")  # Intermediate distance to return repeatedly.
interface = rospy.get_param("comms_interface_enable")  # Interface used to control the communication status in simulations.
odom_topic = rospy.get_param("odometry")  # Topic used to obtain information about the position of the robot.
frame = rospy.get_param("child_frame_id")  # Transform which contains the robot pose.

# Initiate the buffer used to store robot's positions.
position_buffer = []


# Function to change communication status  in simulations.
def communication_state(a, pub_signal):
    if a == "active":
        pub_signal.publish(False)
    elif a == "inactive":
        pub_signal.publish(True)


# Function to receive communication status from robot
def callback_signal(data):
    global lost_signal
    lost_signal = data.data


# Class defined to create the interface utilized to alter the status of communication in simulations.
class Window(threading.Thread):

    def __init__(self, pub_signal, lost_signal):
        threading.Thread.__init__(self)
        self.pub_signal = pub_signal
        self.lost_signal = lost_signal
        self.start()

    def change_text(self, string):
        self.window.msg2['text'] = string

    def run(self):
        self.window = Tk()
        self.window.msg1 = Label(self.window, width=200, text="Escolha o estado da comunicação:")
        self.window.msg1.pack()
        self.window.msg1.place(x=45, y=20)
        self.window.msg2 = Label(self.window, width=200, text="(Estado Atual: Ativo)")
        self.window.msg2.pack()
        self.window.msg2.place(x=85, y=40)
        self.window.geometry("300x150+900+150")
        self.window.title("Comunicação")
        self.window.style = Style()
        self.window.style.theme_use("default")
        ActiveButton = Button(self.window, text="Sinal Ativo", command=lambda: [self.pub_signal.publish(False), self.change_text("(Estado Atual: Ativo)")])
        InactiveButton = Button(self.window, text="Sinal Inativo", command=lambda: [self.pub_signal.publish(True), self.change_text("(Estado Atual: Inativo)")])
        ActiveButton.pack(side=LEFT, padx=45, pady=(60, 15))
        InactiveButton.pack(side=LEFT, pady=(60, 15))
        self.window.mainloop()


# Callback to get the pose of the robot
def callback_pose_tf(data):
    global pos, rpy

    for T in data.transforms:
        # Choose the transform of the EspeleoRobo
        if (T.child_frame_id == "base_link"):

            # Get the orientation
            x_q = T.transform.rotation.x
            y_q = T.transform.rotation.y
            z_q = T.transform.rotation.z
            w_q = T.transform.rotation.w
            euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
            theta_n = euler[2]

            # Get the position
            pos[0] = T.transform.translation.x
            pos[1] = T.transform.translation.y
            pos[2] = T.transform.translation.z
            rpy = euler

    return
# ---------------------------------------------------------


# Stores robot's current position.
def get_position():
    global position_buffer
    position_buffer.append((pos[0], pos[1]))

# --------------------------------------------------------


# Function to send a array of markers, representing the curve, to rviz
def send_curve_to_rviz(position_buffer, pub_rviz):

    # Create messsage
    points_marker = MarkerArray()
    marker = Marker()
    # Iterate over the points
    for k in range(len(position_buffer)):
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = k
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Size of sphere
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        # Color and transparency
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # Pose
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = position_buffer[k][0]
        marker.pose.position.y = position_buffer[k][1]
        marker.pose.position.z = 0.1
        # Append marker to array
        points_marker.markers.append(marker)

    pub_rviz.publish(points_marker)

    return (points_marker)
# ----------------------------------------------------------


# Function to create a message of the type polygon, which will carry the points of the curve
def create_traj_msg(position_buffer):

    # Create 'Polygon' message (array of messages of type 'Point')
    traj_msg = Polygon()
    p = Point()
    position_buffer.reverse()
    length = len(position_buffer)
    for k in range(length):
        # Create point
        p = Point()
        # Atribute values
        p.x = position_buffer[k][0]
        p.y = position_buffer[k][1]
        p.z = 0.0
        # Append point to polygon
        traj_msg.points.append(p)

    return traj_msg
# -------------------------------------------------------


def position_return():
    rospy.init_node('return', anonymous=True)

    # Topic where the next trajectory to be followed is published.
    pub_traj = rospy.Publisher("/espeleo/traj_points", Path, queue_size=1)
    # Topic to send trajectory to rviz.
    pub_rviz_curve = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
    # Topic where the odometry of the robot is received from.
    rospy.Subscriber(odom_topic, TFMessage, callback_pose_tf)
    # Flag to receive what is the current status of communication.
    rospy.Subscriber("/flag/signal", Bool, callback_signal)
    # Topic used to change the status of communication (Only used in simulations).
    pub_signal = rospy.Publisher("/flag/signal", Bool, queue_size=5)
    # Topic used to clear the current trajectory that the robot is following.
    pub_clear_traj = rospy.Publisher("/espeleo/vecfield_enable", Bool, queue_size=5)

    rate = rospy.Rate(freq)

    global lost_signal
    lost_signal = False

    # Configuration of the trajectory message sent to the robot.
    traj_m = Path()
    traj_m.closed_path_flag = False
    traj_m.insert_n_points = 0
    traj_m.filter_path_n_average = 3

    # Small delay so system can be properly set up.
    rospy.sleep(0.5)

    # Variables used to store robot's previous stored position.
    x_n_previous = pos[0]
    y_n_previous = pos[1]

    # Variables used to store coordinates of the final destination in trajectory to be followed.
    x0 = 0.0
    y0 = 0.0

    # Variables used to manage intermediate stops.
    i_aux1 = 0
    i_aux2 = 0
    dist_aux = 0

    # Option to turn communication interface on/off.
    if interface:
        global window
        Window(pub_signal, lost_signal)

    # Auxiliar variables used in routine control.
    initial_log = True
    intermediate_stop = True
    final_log = True
    control = False

    # Stores distance in meters contained in the buffer
    buffer_size = 0.0

    while not rospy.is_shutdown():

        # Routine done until the robot loses communication, saving the position of the trajectory followed so far.
        while (not lost_signal):
            # Reseting of variables in case of reestablishment of communication, allowing data to be stored again.
            if control:
                pub_clear_traj.publish(True)
                initial_log = True
                intermediate_stop = True
                final_log = True
                control = False
                try:
                    del position_buffer[:]
                    buffer_size = 0
                    x_n_previous = pos[0]
                    y_n_previous = pos[1]
                    rospy.loginfo("-----------------------------------------")
                    rospy.loginfo("Connection reestablished")
                    rospy.loginfo("Cleaning the buffer")
                except Exception:
                    print('Buffer already empty')
                break

            # Computes difference between current robot's position and previous one stored.
            distance_variation = sqrt((pos[0] - x_n_previous)**2 + (pos[1] - y_n_previous)**2)

            # Add current robot's position to the buffer if the difference between it and previous one is bigger than delta.
            if (distance_variation > delta):
                x_n_previous = pos[0]
                y_n_previous = pos[1]
                buffer_size = buffer_size + distance_variation

                # Deletes first position of buffer if its already full and adds new information
                if (buffer_size > size):
                    position_buffer.pop(0)
                    get_position()
                else:
                    get_position()

            else:
                # buffer_size = buffer_size + distance_variation
                continue

        # Portion executed when the robot loses communication.
        if lost_signal:
            control = True

            # Routine done once everytime the robot loses communication.
            if(initial_log):
                buffer_size = 0.0
                rospy.loginfo("-----------------------------------------")
                rospy.loginfo("Lost Signal")
                rospy.loginfo("Total distance to return: %s meters", size)
                rospy.loginfo("Intermediate distance: %s meters", intermediate_dist)
                try:
                    # x0 e y0 sao as coord. do ponto de retorno
                    x0, y0 = (position_buffer[0][0], position_buffer[0][1])
                    i_aux1 = len(position_buffer) - 1
                except Exception:
                    print('A problem occurred creating the buffer')
                initial_log = False

            # Calculates the portion of the buffer that corresponds to the distance between intermediate points and sends it to the robot.
            if(intermediate_stop):
                i_aux2 = i_aux1 - 1
                dist_aux = 0
                while(dist_aux < intermediate_dist and i_aux1 > 0):
                    i_aux1 -= 1
                    dist_aux = sqrt((position_buffer[i_aux1][0] - position_buffer[i_aux2][0])**2 + (position_buffer[i_aux1][1] - position_buffer[i_aux2][1])**2)
                traj_m.path = create_traj_msg(position_buffer[i_aux1:i_aux2])
                pub_traj.publish(traj_m)
                time.sleep(1.0)
                send_curve_to_rviz(position_buffer[i_aux1:i_aux2], pub_rviz_curve)
                intermediate_stop = False

            if final_log:
                # Compute distance between current robot position and next intermeadiate stop.
                intermediate_distance = sqrt((pos[0] - position_buffer[i_aux1][0])**2 + (pos[1] - position_buffer[i_aux1][1])**2)
                # Compute distance between current robot position and final destination.
                distance = sqrt((pos[0] - x0)**2 + (pos[1] - y0)**2)

            # Check constinuosly if the robot has reached next intermeadiate stop.
            if(i_aux1 != 0 and intermediate_distance < 1.0*tolerance):
                pub_clear_traj.publish(True)
                rospy.loginfo("-----------------------------------------")
                rospy.loginfo("Intermediate point reached!")
                rospy.loginfo("Waiting 10 seconds!")
                now = time.time()
                future = now + 10
                while time.time() < future:
                    if not lost_signal:
                        break
                if time.time() >= future:
                    rospy.loginfo("-----------------------------------------")
                    rospy.loginfo("Going to next stop point.")
                    intermediate_stop = True
                    dist_aux = 0

            # Check constinuosly if the robot has reached final destination.
            if distance < 1.0*tolerance and final_log:
                pub_clear_traj.publish(True)
                final_log = False
                rospy.loginfo("-----------------------------------------")
                rospy.loginfo("Goal reached")
                # Cleaning buffer.
                try:
                    del position_buffer[:]
                    buffer_size = 0
                    x_n_previous = pos[0]
                    y_n_previous = pos[1]
                    rospy.loginfo("Cleaning the buffer")
                except Exception:
                    print('Buffer already empty')

        rate.sleep()


if __name__ == '__main__':

    # Frequency of computation
    global freq
    freq = 30.0  # Hz

    global pos, rpy
    pos = [0, 0, 0]
    rpy = [0, 0, 0]

    try:
        position_return()
    except rospy.ROSInterruptException:
        pass
