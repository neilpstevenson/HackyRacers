from time import sleep
from approxeng.input.selectbinder import ControllerResource
import pi_servo_hat
import time

# Initialize Constructor
servo_hat = pi_servo_hat.PiServoHat()

# Restart Servo Hat (in case Hat is frozen/locked)
servo_hat.restart()
sleep(0.5)
# Make more responsive
servo_hat.set_pwm_frequency(200)
        
while True:
    try:
        with ControllerResource() as joystick:
            print('Found a joystick and connected')
            while joystick.connected:
                right_x = joystick.rx
                left_x = joystick.lx
                left_y = joystick.ly
                print(f"x={left_x}, y={left_y}, steer={right_x}")
                
                # update the steering servo
                servo_hat.move_servo_position(0, 55 - right_x * 25)
                # update the speed ESC
                servo_hat.move_servo_position(1, left_y * 15 + 32)
                
                sleep(0.02)
                
        # Joystick disconnected...
        print('Connection to joystick lost')
    except IOError:
        # No joystick found, wait for a bit before trying again
        print('Unable to find any joysticks')
        sleep(1.0)
