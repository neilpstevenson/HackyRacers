import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac
from time import sleep
from approxeng.input.selectbinder import ControllerResource
import pi_servo_hat
import time
from simple_pid import PID

print(dir(ac))

servo_hat = pi_servo_hat.PiServoHat()
# Restart Servo Hat (in case Hat is frozen/locked)
servo_hat.restart()
sleep(0.5)
# Make more responsive
servo_hat.set_pwm_frequency(200)

# PID
pid = PID(1.0, 0.0, 0.1, output_limits=(-1.0,1.0), sample_time=0.02, setpoint = 1.0)

MAX_DISTANCE = 4

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

followRect = UserRect()

def on_mouse(event, x, y, flags, param):
    global selectRect,followRect
    
    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        selectRect.start_x = x - 4 if x - 4 > 0 else 0
        selectRect.start_y = y - 4 if y - 4 > 0 else 0
        selectRect.end_x = x + 4 if x + 4 < 240 else 240
        selectRect.end_y=  y + 4 if y + 4 < 180 else 180
    else:
        followRect.start_x = x - 4 if x - 4 > 0 else 0
        followRect.start_y = y - 4 if y - 4 > 0 else 0
        followRect.end_x = x + 4 if x + 4 < 240 else 240
        followRect.end_y = y + 4 if y + 4 < 180 else 180
        
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
    
    target_distance = 1.0
    
    while True:
        try:
            frame = cam.requestFrame(200)
            if frame != None:
                depth_buf = frame.getDepthData()
                amplitude_buf = frame.getAmplitudeData()
                cam.releaseFrame(frame)
                amplitude_buf*=(255/1024)
                amplitude_buf = np.clip(amplitude_buf, 0, 255)

                cv2.imshow("preview_amplitude", amplitude_buf.astype(np.uint8))
                distance = np.mean(depth_buf[selectRect.start_y:selectRect.end_y,selectRect.start_x:selectRect.end_x])
                print("select Rect distance:", distance)
                result_image = process_frame(depth_buf,amplitude_buf)
                result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
                cv2.rectangle(result_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
                cv2.rectangle(result_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
        
                cv2.imshow("preview",result_image)

                key = cv2.waitKey(1)
                if key == ord("q"):
                    exit_ = True
                    cam.stop()
                    cam.close()
                    sys.exit(0)
                
                # Update the servo
                if(not np.isnan(distance)):
                    #steer = min(max(distance - target_distance, -1.0), 1.0)
                    steer = pid(distance)
                    print(f"steer = {steer}")
                    
                    # update the steering servo
                    servo_hat.move_servo_position(0, 55 - steer* 25)
                    
                # update the speed ESC
                #servo_hat.move_servo_position(1, left_y * 15 + 32)
                
        except IOError:
            print('ERROR exception - Check Servo pHAT powered up')
            sleep(1.0)