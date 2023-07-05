from dronekit import connect,VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil


# Connect to the Vehicle (in this case a simulator running the same computer)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)



def condition_yaw(vehicle, heading, relative, clock_wise):
    # 使用相对角度或绝对方位
    if relative:
        isRelative = 1
    else:
        isRelative = 0

    # 若使用相对角度，则进行顺时针或逆时针转动
    if clock_wise:
        direction = 1  # "heading"所对应的角度将被加和到当前朝向角
    else:
        direction = -1 # "heading"所对应的角度将被从当前朝向角减去
    if not relative:
        direction = 0

    # 生成CONDITION_YAW命令
    msg = vehicle.message_factory.command_long_encode(
                    0, 0,       # target system, target component
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                    0,          # confirmation
                    heading,    # param 1, yaw in degrees
                    0,          # param 2, yaw speed (not used)
                    direction,  # param 3, direction
                    isRelative, # param 4, relative or absolute degrees
                    0, 0, 0)    # param 5-7, not used
    # 发送指令
    vehicle.send_mavlink(msg)
    vehicle.flush()
