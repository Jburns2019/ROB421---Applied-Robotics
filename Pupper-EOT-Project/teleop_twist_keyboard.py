# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import threading
import time

import control_launch
import exploring
import path_planning
import numpy as np
import matplotlib.pyplot as plt
from multiprocessing import Process

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

#Manually set for demo2. Enter in feet.
target = (-2, 5)

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

Use the space bar to take a 1.5 minute snapshot of the suroundings. Stoped in map.pgm.
Use g to go to the hardcoded location.

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:    
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    movement_queue = []
    try:
        print(msg)
        autonomous = False
        key = ''
        direction, duration = ('Forward', 0)
        while True:
            if not autonomous:
                key = getKey(settings)
            else:
                key = '_'

            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                
            elif key == ' ':
                print('Keyboard unavailable for the next 90 seconds.')
                control_launch.take_snapshot()

            elif key == 'g':
                im, im_thresh = path_planning.open_image("map.pgm")
                all_unseen = exploring.find_all_possible_goals(im_thresh)
                connected_groups = exploring.find_all_connected_pix(all_unseen)
                intersections, lines = exploring.find_intersections(im_thresh, connected_groups)
                robot_start_loc = exploring.get_robot_pos(intersections)

                im = np.fliplr(im)
                im_thresh = np.fliplr(im_thresh)
                robot_start_loc = (im.shape[1]-robot_start_loc[0], robot_start_loc[1])
                print(robot_start_loc)
                
                zoom = 1
                all_unseen = exploring.find_all_possible_goals(im_thresh)
                # best_unseen = exploring.find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)
                pix_target = (exploring.convert_ft_to_px(target[0]), exploring.convert_ft_to_px(target[1]))
                best_unseen = (int(robot_start_loc[0]+pix_target[0]), int(robot_start_loc[1]+pix_target[1]))
                exploring.plot_with_explore_points(im, im_thresh, zoom, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

                plt.savefig('target.png')
                
                path = path_planning.search_algorithm(im_thresh, robot_start_loc, [best_unseen])
                path_planning.plot_with_path(im, im_thresh, zoom, robot_loc=robot_start_loc, goal_loc=best_unseen, path=path)

                plt.savefig('pathing.png')

                movement_queue = path_planning.get_instructions(path)
                autonomous = True
            elif len(movement_queue) > 0:
                instruction = movement_queue.pop(0)
                
                direction, duration = instruction.split()
                
                key = 'i'
                if direction == 'Forward':
                    key = 'i'
                    duration = exploring.convert_px_to_time(int(duration))
                elif direction == 'Turn-left':
                    key = 'j'
                    duration = exploring.convert_angle_to_time(int(duration))
                elif direction == 'Turn-right':
                    key = 'l'
                    duration = exploring.convert_angle_to_time(int(duration))
               
                print(direction, duration)
 
                x, y, z, th = moveBindings[key]
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0

                if (key == '\x03'):
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()
            
            
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

            if autonomous:
                time.sleep(duration)

            if len(movement_queue) == 0 and x == 0.0 and y == 0.0 and z == 0.0 and th == 0.0:
                autonomous = False

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
