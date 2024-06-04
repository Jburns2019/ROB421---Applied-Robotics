import subprocess
import time

def launch_command():
    p = subprocess.Popen("exec ros2 launch mini_pupper_slam slam.launch.py > /dev/null", shell=True)
    print("Started recording lidar hits for occupancy grid.")

    time.sleep(90)

    print("Stoped recording lidar hits.")
    save_screenshot = subprocess.Popen("ros2 run nav2_map_server map_saver_cli -f ~/map", shell=True)
    save_screenshot.wait()

    p.kill()

    print("Killing rviz now.")
    
    remove_rviz_apps = subprocess.Popen("killall rviz2", shell=True)
    remove_rviz_apps.wait()

    #subprocess.Popen("gio open map.pgm", shell=True)

launch_command()
