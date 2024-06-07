running_on_pupper = True
try:
    from UDPComms import Publisher
except:
    running_on_pupper = False

import time

# drive_pub = Publisher(8830) = controls movement of pupper (basically mode 1)
# arm_pub = Publisher(8410) = controls more movements of upper (mode 2)
# mode 2 is what you can do when pupper is not in trot mode when using a controller.
drive_pub = None
if running_on_pupper:
    drive_pub = Publisher(8830)
# arm_pub = Publisher(8410)
# L1 = activate/disactivate
# R1 = transition between Rest mode and Trot mode.
# circle = dance or hold for 3 seconds to turn off system
# trinagle  = NOTHING
# X = jump
# L2 = nothing
# R2 = Nothing
# The range for the following are form (-1, 1)
# ly = forward or backwards
# lx = left or right
# rx = turn left or right (pitch)
# ry = pitches the robot forward

is_on = False
is_trotting = False
verbose = True

def wait(times=1.0):
    start_time = time.time()
    current_time = time.time()
    while current_time-start_time < times:
        current_time = time.time()

def make_cmd(command = None, toggle_activation=False, toggle_trot=False, jump=False, l2=False, r2=False, y=0, x=0, xy_yaw=0, xy_pitch=0, circle=False, triangle=False, dpadx=0, dpady=0, message_rate=20):
    if command == None:
        command = {}
    
    command["L1"] = int(toggle_activation)
    command["R1"] = int(toggle_trot)
    command["x"] = int(jump)
    command["circle"] = int(circle)
    command["triangle"] = int(triangle)
    command["L2"] = int(l2)
    command["R2"] = int(r2)
    command["ly"] = y
    command["lx"] = x
    command["rx"] = xy_yaw
    command["message_rate"] = message_rate
    command["ry"] = xy_pitch
    command["dpady"] = dpady
    command["dpadx"] = dpadx

    return command

def activate():
    global is_on
    if not is_on:
        send_command(make_cmd(toggle_activation=True))
        wait(1)
        send_command(make_cmd())
        is_on = True

def deactivate():
    global is_on, is_trotting

    toggle_trot = False
    if is_trotting:
        toggle_trot = True

    if is_on:
        send_command(make_cmd(toggle_activation=True, toggle_trot=toggle_trot))
        wait(1)
        send_command(make_cmd())
        is_on = False
        is_trotting = False

def start_trotting():
    global is_trotting
    if not is_trotting:
        send_command(make_cmd(toggle_trot=True))
        is_trotting = True

        wait(.02)

def move(dir: str='None'):
    global is_trotting

    if dir == 'None':
        stop_moving()
    else:
        x = 0
        if 'left' in dir:
            x = -1
        elif 'right' in dir:
            x = 1
        
        y = 0
        if 'forward' in dir:
            y = 1
        elif 'back' in dir:
            y = -1

        if not is_trotting:
            start_trotting()

        send_command(make_cmd(x=x, y=y))

def turn(dir: str='None'):
    global is_trotting

    if dir == 'None':
        stop_moving()
    else:
        xy_yaw = 0
        if 'right' in dir:
            xy_yaw = 1
        elif 'left' in dir:
            xy_yaw = -1
        
        y = 0
        if 'forward' in dir:
            y = 1
        elif 'back' in dir:
            y = -1

        toggle_trot = False
        if not is_trotting:
            start_trotting()
        
        send_command(make_cmd(y=y, xy_yaw=xy_yaw))

def stop_moving():
    global is_trotting

    toggle_trot = False
    if is_trotting:
        toggle_trot = True
    
    send_command(make_cmd(toggle_trot=toggle_trot))
    is_trotting = False

def send_command(command):
    if running_on_pupper:
        drive_pub.send(command)

    if verbose or not running_on_pupper:
        print(command)

def turn_by_degrees(deg=360, dir='left'):
    count = int(deg/360.0*250.0)
    if dir == 'right':
        count = int(deg/360.0*280.0)
    
    for i in range(count):
        turn(dir)
        wait(.02)

def move_by_feet(amount=3, dir='forward'):
    count = int(amount/3.0*450.0)
    
    for i in range(count):
        # if i % 18 == 0 and dir == 'forward':
        #     turn('forward-left')
        # else:
        move(dir)
        wait(.02)

# TODO: create functions that allow the robot to move around (forward,back,right,left,....)
# Remember: The inputs are mainly digital except for the lx,ly and rx,ry controls.
# The digital inputs do not reset after being call unless you design them to! (i.e., if you press L1 it will remaind press)
# Each action needs to be press once then can be ignored (you don't have to keep L1 as 1 you can create an activate function the forget about it)   
# TODO: make the robot move through the racing track
if __name__ == "__main__":
    activate()
    wait(.02)
    
    move_by_feet(3, 'forward')
    # turn_by_degrees(70, 'left')
    # move_by_feet(1.8, 'forward')
    # turn_by_degrees(60, 'right')
    
    stop_moving()
    wait(.02)
    deactivate()
    