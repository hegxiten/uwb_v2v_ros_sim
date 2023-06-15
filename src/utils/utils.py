import os
import psutil


def packet_loss_probability(dist, max_range):
    # The higher the distance, the higher the probability of packet loss. If the distance is greater than max_range,
    # then the packet is lost.
    if dist > max_range:
        return 1.0
    return dist / max_range


def kill_gazebo():
    for proc in psutil.process_iter():
        # check whether the process name matches
        if proc.name() == "gzserver" or proc.name() == "gzclient":
            proc.kill()


def shutdown_ros_node():
    os.system('rosnode kill -a')


def kill_ros_master():
    os.system('killall -9 rosmaster')
    os.system('pkill -f ros')

