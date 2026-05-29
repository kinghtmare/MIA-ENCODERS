#!/usr/bin/env python3
"""
encoder_listener_v2.py
----------------------
Displays always-positive encoder data + xAxis direction vector.

Topics:
  /encoder/count     Int32   — positive count, resets on direction change
  /encoder/distance  Float32 — positive cm, resets on direction change
  /encoder/velocity  Float32 — positive cm/s
  /encoder/xaxis     Int32   — +1 Forward | -1 Reverse
"""

import rospy
from std_msgs.msg import Int32, Float32
import os

class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    CYAN   = "\033[96m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    GRAY   = "\033[90m"
    WHITE  = "\033[97m"

state = {
    "count":    0,
    "distance": 0.0,
    "velocity": 0.0,
    "xaxis":    0,        # 0 = unknown, +1 = forward, -1 = reverse
}

def cb_count(msg):    state["count"]    = msg.data
def cb_distance(msg): state["distance"] = msg.data
def cb_velocity(msg): state["velocity"] = msg.data
def cb_xaxis(msg):    state["xaxis"]    = msg.data

def direction_display(xaxis):
    if xaxis == 1:
        return f"{C.GREEN}{C.BOLD}  FORWARD  ▶▶▶{C.RESET}"
    elif xaxis == -1:
        return f"{C.RED}{C.BOLD}  REVERSE  ◀◀◀{C.RESET}"
    else:
        return f"{C.GRAY}  UNKNOWN  ---{C.RESET}"

def xaxis_display(xaxis):
    if xaxis == 1:
        return f"{C.GREEN}+1{C.RESET}"
    elif xaxis == -1:
        return f"{C.RED}-1{C.RESET}"
    else:
        return f"{C.GRAY} 0{C.RESET}"

def render():
    os.system("clear")
    c = state["count"]
    d = state["distance"]
    v = state["velocity"]
    x = state["xaxis"]

    print(f"{C.BOLD}{C.WHITE}{'═'*48}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}    ESP32 ENCODER — LIVE MONITOR  v2{C.RESET}")
    print(f"{C.BOLD}{C.WHITE}{'═'*48}{C.RESET}")
    print()
    print(f"  {C.BOLD}Direction {C.RESET}:{direction_display(x)}")
    print(f"  {C.BOLD}X-Axis    {C.RESET}: {xaxis_display(x)}  {C.GRAY}(+1 Fwd | -1 Rev){C.RESET}")
    print()
    print(f"{C.GRAY}  ── Values reset to 0 on every direction change ──{C.RESET}")
    print()
    print(f"  {C.BOLD}Count     {C.RESET}: {C.WHITE}{c:>8d}{C.RESET}  {C.GRAY}ticks{C.RESET}")
    print(f"  {C.BOLD}Distance  {C.RESET}: {C.WHITE}{d:>10.3f}{C.RESET}  {C.GRAY}cm{C.RESET}")
    print(f"  {C.BOLD}Velocity  {C.RESET}: {C.WHITE}{v:>10.3f}{C.RESET}  {C.GRAY}cm/s{C.RESET}")
    print()
    print(f"{C.BOLD}{C.WHITE}{'─'*48}{C.RESET}")
    print(f"{C.GRAY}  Ctrl+C to exit{C.RESET}")

def main():
    rospy.init_node("encoder_listener_v2", anonymous=False)

    rospy.Subscriber("/encoder/count",    Int32,   cb_count)
    rospy.Subscriber("/encoder/distance", Float32, cb_distance)
    rospy.Subscriber("/encoder/velocity", Float32, cb_velocity)
    rospy.Subscriber("/encoder/xaxis",    Int32,   cb_xaxis)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        render()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
