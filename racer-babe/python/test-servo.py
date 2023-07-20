from time import sleep
from approxeng.input.selectbinder import ControllerResource
import pi_servo_hat
import time

# Initialize Constructor
servo_hat = pi_servo_hat.PiServoHat()

# Restart Servo Hat (in case Hat is frozen/locked)
servo_hat.restart()
sleep(0.5)
        
while True:
    try:
        with ControllerResource() as joystick:
            print('Found a joystick and connected')
            while joystick.connected:
                # Do stuff with your joystick here!
                # ....
                # ....
                left_x = joystick.lx
                left_y = joystick.ly
                #print(f"x={left_x}, y={left_y}")
                
                # update the steering servo
                servo_hat.move_servo_position(0, 55 - left_x * 25)
                # update the speed ESC
                servo_hat.move_servo_position(1, 55 - left_y * 25)
                
                sleep(0.02)
                
        # Joystick disconnected...
        print('Connection to joystick lost')
    except IOError:
        # No joystick found, wait for a bit before trying again
        print('Unable to find any joysticks')
        sleep(1.0)
