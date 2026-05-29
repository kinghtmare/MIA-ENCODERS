#!/usr/bin/env python3
"""
encoder_listener.py
-------------------
ROS 1 subscriber node — displays encoder data published by the ESP32.

Run in a SEPARATE terminal AFTER the rosserial bridge is running:

  Terminal 1 (roscore):
      roscore

  Terminal 2 (rosserial bridge):
      rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

  Terminal 3 (this node):
      python3 encoder_listener.py
      # OR if installed as a ROS package:
      rosrun your_package encoder_listener.py

Subscribed topics:
  /encoder/count     (std_msgs/Int32)   — raw tick count
  /encoder/distance  (std_msgs/Float32) — cumulative distance in cm
  /encoder/velocity  (std_msgs/Float32) — instantaneous velocity in cm/s
"""

import rospy
from std_msgs.msg import Int32, Float32
import sys
import os

# =============================================
# ANSI COLOR HELPERS — makes terminal output readable
# =============================================
class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    CYAN   = "\033[96m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    GRAY   = "\033[90m"
    WHITE  = "\033[97m"

# =============================================
# SHARED STATE — updated by callbacks, printed by display loop
# =============================================
state = {
    "count":    0,
    "distance": 0.0,
    "velocity": 0.0,
    "count_updated":    False,
    "distance_updated": False,
    "velocity_updated": False,
}

# =============================================
# SUBSCRIBER CALLBACKS
# =============================================
def callback_count(msg):
    state["count"]           = msg.data
    state["count_updated"]   = True

def callback_distance(msg):
    state["distance"]          = msg.data
    state["distance_updated"]  = True

def callback_velocity(msg):
    state["velocity"]          = msg.data
    state["velocity_updated"]  = True

# =============================================
# DISPLAY HELPERS
# =============================================
def clear_line():
    sys.stdout.write("\033[K")   # erase to end of line

def print_header():
    os.system("clear")
    print(f"{C.BOLD}{C.WHITE}{'='*52}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}   ESP32 ENCODER — LIVE DATA MONITOR{C.RESET}")
    print(f"{C.BOLD}{C.WHITE}{'='*52}{C.RESET}")
    print(f"{C.GRAY}  Topics: /encoder/count | /encoder/distance | /encoder/velocity{C.RESET}")
    print(f"{C.BOLD}{C.WHITE}{'─'*52}{C.RESET}")

def direction_indicator(velocity):
    """Return a simple direction string based on velocity sign."""
    if abs(velocity) < 0.1:
        return f"{C.GRAY}STOPPED ●{C.RESET}"
    elif velocity > 0:
        return f"{C.GREEN}FORWARD ▶{C.RESET}"
    else:
        return f"{C.RED}REVERSE ◀{C.RESET}"

def format_value(value, fmt, unit, updated, warn_threshold=None):
    """Color a value green if just updated, yellow if above warn threshold."""
    color = C.GREEN if updated else C.WHITE
    if warn_threshold and abs(value) > warn_threshold:
        color = C.YELLOW
    return f"{color}{value:{fmt}}{C.RESET} {C.GRAY}{unit}{C.RESET}"

# =============================================
# MAIN NODE
# =============================================
def encoder_listener():
    rospy.init_node("encoder_listener", anonymous=False)
    rospy.loginfo("encoder_listener started. Waiting for ESP32 data...")

    rospy.Subscriber("/encoder/count",    Int32,   callback_count)
    rospy.Subscriber("/encoder/distance", Float32, callback_distance)
    rospy.Subscriber("/encoder/velocity", Float32, callback_velocity)

    rate = rospy.Rate(10)   # refresh display at 10 Hz (same as ESP32 publish rate)

    print_header()
    print(f"\n{C.YELLOW}  Waiting for messages...{C.RESET}\n")

    received_first = False

    while not rospy.is_shutdown():
        any_updated = (
            state["count_updated"] or
            state["distance_updated"] or
            state["velocity_updated"]
        )

        if any_updated and not received_first:
            received_first = True
            print_header()

        if received_first:
            # Re-print the data block in place (move cursor up 7 lines after first render)
            # Use a simple full-refresh every cycle — clean and reliable
            print_header()

            count    = state["count"]
            distance = state["distance"]
            velocity = state["velocity"]

            print()
            print(f"  {C.BOLD}Raw Count   {C.RESET}: "
                  f"{format_value(count, 'd', 'ticks', state['count_updated'])}")

            print(f"  {C.BOLD}Distance    {C.RESET}: "
                  f"{format_value(distance, '.3f', 'cm', state['distance_updated'])}")

            print(f"  {C.BOLD}Velocity    {C.RESET}: "
                  f"{format_value(velocity, '.3f', 'cm/s', state['velocity_updated'], warn_threshold=50.0)}")

            print()
            print(f"  {C.BOLD}Direction   {C.RESET}: {direction_indicator(velocity)}")
            print()
            print(f"{C.BOLD}{C.WHITE}{'─'*52}{C.RESET}")
            print(f"{C.GRAY}  Press Ctrl+C to exit{C.RESET}")

            # Reset update flags after printing
            state["count_updated"]    = False
            state["distance_updated"] = False
            state["velocity_updated"] = False

        rate.sleep()

    rospy.loginfo("encoder_listener shutting down.")

# =============================================
# ENTRY POINT
# =============================================
if __name__ == "__main__":
    try:
        encoder_listener()
    except rospy.ROSInterruptException:
        pass
