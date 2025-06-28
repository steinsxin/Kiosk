import os
import time
from multiprocessing import Queue
from threading import Thread

import rclpy

from autolife_robot_kiosk.logo_ros_battery import BatteryNode
from autolife_robot_kiosk.logo_ui_display import display_ui
from autolife_robot_kiosk import ASSETS_ROOT, PROGRAM_SETTINGS
from autolife_robot_sdk.utils import get_mac_from_ip

def wait_for_mac(ip, interval=1.0):
    """
    Repeatedly attempts to retrieve the MAC address of the target IP
    until a valid (non-zero) address is obtained.
    """
    while True:
        mac = get_mac_from_ip(ip).replace(":", "")
        if mac and mac != "00000000":
            print(f"[INFO] DEVICE_ID acquired: {mac}")
            return mac
        print(f"[WARN] Waiting for device {ip} to boot...")
        time.sleep(interval)


def launch_ros_node(queue):
    """
    Launches the ROS BatteryNode after obtaining a valid MAC address.
    Runs in a background thread and does not block UI display.
    """
    rclpy.init()

    device_id = wait_for_mac(PROGRAM_SETTINGS['app_settings']['TARGET_IP'])
    node = BatteryNode(queue, device_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """
    Main entry point for launching the UI display and ROS battery node
    in separate threads. The UI starts immediately, while the ROS node
    waits for the target device to boot.
    """
    queue = Queue()

    ui_thread = Thread(target=display_ui, args=(queue,ASSETS_ROOT))
    ui_thread.start()

    ros_thread = Thread(target=launch_ros_node, args=(queue,))
    ros_thread.start()

    ui_thread.join()
    ros_thread.join()


if __name__ == "__main__":
    main()
