import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac
from time import sleep
from approxeng.input.selectbinder import ControllerResource
from approxeng.input.dualshock4 import DualShock4
from approxeng.input.selectbinder import ControllerResource, ControllerRequirement, ControllerNotFoundError
import pi_servo_hat
import time
from simple_pid import PID

print(dir(ac))

MAX_DISTANCE = 4    # Metres
# Camera is 240x180
DEFAULT_REGION_X = 30
DEFAULT_REGION_Y = 70  # From top (70 is approx 240mm above ground at target distance)
AHEAD_REGION_X = 220
AHEAD_REGION_Y = 70  # From top
REGION_WIDTH = 16   # Width in pixels
REGION_HEIGHT = 8   # Height in pixels

WALL_FOLLOW_DISTANCE = 1.0 * 1.4  # Metres to the left, at around 45degs

STEERING_SCALE = 63
STEERING_CENTRE = 110
SPEED_SCALE = 64
SPEED_IDLE = 64

auto_speed = 0.4
speed_deadband = 0.2    # ESC ignore anthing below this

# PID
pid = PID(1.5, 0.0, 0.8, output_limits=(-1.0,1.0), sample_time=0.01, setpoint = WALL_FOLLOW_DISTANCE)


def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame 

class UserRect():
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

selectRect = UserRect()
aheadRect = UserRect()
followRect = UserRect()

def on_mouse(event, x, y, flags, param):
    global selectRect,followRect
    
    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        selectRect.start_x = x - (REGION_WIDTH//2) if x - (REGION_WIDTH//2) > 0 else 0
        selectRect.start_y = y - (REGION_HEIGHT//2) if y - (REGION_HEIGHT//2) > 0 else 0
        selectRect.end_x = x + (REGION_WIDTH//2) if x + (REGION_WIDTH//2) < 240 else 240
        selectRect.end_y=  y + (REGION_HEIGHT//2) if y + (REGION_HEIGHT//2) < 180 else 180
    else:
        followRect.start_x = x - (REGION_WIDTH//2) if x - (REGION_WIDTH//2) > 0 else 0
        followRect.start_y = y - (REGION_HEIGHT//2) if y - (REGION_HEIGHT//2) > 0 else 0
        followRect.end_x = x + (REGION_WIDTH//2) if x + (REGION_WIDTH//2) < 240 else 240
        followRect.end_y = y + (REGION_HEIGHT//2) if y + (REGION_HEIGHT//2) < 180 else 180
        
def usage(argv0):
    print("Usage: python "+argv0+" [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")


if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.open(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)
    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("preview",on_mouse)
    
    filtered_distance = WALL_FOLLOW_DISTANCE

    # Default selected rect
    selectRect.start_x = DEFAULT_REGION_X
    selectRect.start_y = DEFAULT_REGION_Y
    selectRect.end_x = selectRect.start_x + REGION_WIDTH
    selectRect.end_y = selectRect.start_y + REGION_HEIGHT
    # Default forward sense rect
    aheadRect.start_x = AHEAD_REGION_X
    aheadRect.start_y = AHEAD_REGION_Y
    aheadRect.end_x = aheadRect.start_x + REGION_WIDTH
    aheadRect.end_y = aheadRect.start_y + REGION_HEIGHT

    
    while True:
        try:
            servo_hat = pi_servo_hat.PiServoHat()
            # Restart Servo Hat (in case Hat is frozen/locked)
            servo_hat.restart()
            sleep(0.5)
            # Make more responsive
            servo_hat.set_pwm_frequency(200)
            
            mode = 'manual'
            
            with ControllerResource(ControllerRequirement(require_class=DualShock4)) as joystick:
                while True:
                    frame = cam.requestFrame(200)
                    if frame != None:
                        depth_buf = frame.getDepthData()
                        amplitude_buf = frame.getAmplitudeData()
                        cam.releaseFrame(frame)
                        amplitude_buf*=(255/1024)
                        amplitude_buf = np.clip(amplitude_buf, 0, 255)

                        amplitutude_image = amplitude_buf.astype(np.uint8)
                        cv2.rectangle(amplitutude_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
                        cv2.rectangle(amplitutude_image,(aheadRect.start_x,aheadRect.start_y),(aheadRect.end_x,aheadRect.end_y),(128,128,128), 1)
                        cv2.rectangle(amplitutude_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
                        cv2.imshow("preview_amplitude", amplitutude_image)
                        
                        distance = np.mean(depth_buf[selectRect.start_y:selectRect.end_y,selectRect.start_x:selectRect.end_x])
                        ahead_distance = np.mean(depth_buf[aheadRect.start_y:aheadRect.end_y,aheadRect.start_x:aheadRect.end_x])
                        print(f"Left distance: {distance}mm, Ahead: {ahead_distance}mm")
                        result_image = process_frame(depth_buf,amplitude_buf)
                        result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
                        cv2.rectangle(result_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
                        cv2.rectangle(result_image,(aheadRect.start_x,aheadRect.start_y),(aheadRect.end_x,aheadRect.end_y),(128,128,128), 1)
                        cv2.rectangle(result_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
                
                        cv2.imshow("preview",result_image)

                        key = cv2.waitKey(1)
                        if key == ord("q"):
                            exit_ = True
                            cam.stop()
                            cam.close()
                            sys.exit(0)

                        # To prevent ploughing into obstructions ahead,
                        # use the ahead distance if it's nearer than the wall distance
                        min_distance = min(distance,ahead_distance)

                        # Update the steering servo
                        if(mode != 'manual' and not np.isnan(distance)):
                            # Simple filter
                            filtered_distance = 0.9*filtered_distance + 0.1*min_distance
                            # Apply PID control
                            steer = pid(filtered_distance)
                            # Adjust accoring to current speed
                            #steer = steer * (1 - (speed - speed_deadband)*0.5)
                            
                            # update the steering servo
                            print(f"steer = {steer}, dist = {filtered_distance}")
                            servo_hat.move_servo_position(0, STEERING_CENTRE - steer * STEERING_SCALE, 180)
                        else:
                            # Manual update
                            right_x = joystick.rx
                            servo_hat.move_servo_position(0, STEERING_CENTRE - right_x * STEERING_SCALE, 180)
                            
                        # update the speed ESC
                        if(joystick.connected):
                            if(mode == 'auto'):
                                # Fixed speed, but slow down when cornering
                                speed = auto_speed * (1.0-abs(steer/3))
                            else:
                                # Manual control
                                left_y = joystick.ly
                                speed = left_y
 
                            servo_hat.move_servo_position(1, speed * SPEED_SCALE + SPEED_IDLE, 180)
                            print(f"speed = {speed}")
                                
                            # Check mode switch
                            presses = joystick.check_presses()
                            if(presses.square):
                                # Switch between manual and auto
                                if(mode == 'manual'):
                                    mode = 'steer'
                                    pid.reset()
                                    filtered_distance = min_distance
                                    joystick.set_leds(hue=0.66) # Blue
                                    joystick.rumble(milliseconds=200)
                                else:
                                    mode = 'manual'
                                    joystick.set_leds(hue=0.33) # Green
                                    joystick.rumble(milliseconds=200)
                                print(f'MODE now {mode}')
                            elif(presses.circle):
                                # Switch between full auto and steer only
                                if(mode == 'steer'):
                                    mode = 'auto'
                                    joystick.set_leds(hue=0.0) # Red
                                    joystick.rumble(milliseconds=200)
                                elif(mode == 'auto'):
                                    mode = 'steer'
                                    joystick.set_leds(hue=0.66) # Blue
                                    joystick.rumble(milliseconds=200)
                                print(f'MODE now {mode}')
                            elif(presses.dup and auto_speed < 1):
                                auto_speed += 0.03
                            elif(presses.ddown and auto_speed > 0.1):
                                auto_speed -= 0.03
                                
                        else:
                            print("Jotstick lost - stopping")
                            servo_hat.move_servo_position(1, SPEED_IDLE, 180)
                            break
                     
        except IOError as e:
            print('ERROR exception - Check Controller and Servo pHAT powered up')
            print(e)
            try:
                # Stop
                servo_hat.move_servo_position(1, speed_idle, 180)
            except:
                pass
            sleep(1.0)
