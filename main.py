# Importing the VEX library
import vex

# Initialize the VEX robot's brain, motors, and sensors
brain = vex.Brain()
left_motor = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO_18_1, False)
right_motor = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO_18_1, True)
additional_motor1 = vex.Motor(vex.Ports.PORT9, vex.GearSetting.RATIO_18_1, False)
additional_motor2 = vex.Motor(vex.Ports.PORT10, vex.GearSetting.RATIO_18_1, True)
distance_sensor = vex.DistanceSensor(vex.Ports.PORT3)

# Constants for the PIL loop
kP = 0.5  # Proportional gain
kI = 0.01 # Integral gain

# Target distance (in mm)
target_distance = 500  # Example: 500 mm

# Variables for the loop
integral = 0
error = 0
previous_error = 0
loop_time = 0.02  # Time between loop iterations (20ms)

# Function to calculate motor power based on PIL
def calculate_motor_power(target, current):
    global integral, error, previous_error

    # Calculate error
    error = target - current

    # Calculate integral
    integral += error * loop_time

    # Calculate motor power
    power = (kP * error) + (kI * integral)

    # Clamp power to max motor range (-100 to 100)
    power = max(min(power, 100), -100)

    return power

# Main autonomous function
def autonomous():
    while True:
        # Get the current distance from the distance sensor
        current_distance = distance_sensor.object_distance_mm()

        # Calculate motor power using PIL
        motor_power = calculate_motor_power(target_distance, current_distance)

        # Apply motor power to drive the robot
        left_motor.spin(vex.DirectionType.FWD, motor_power, vex.VelocityUnits.PCT)
        right_motor.spin(vex.DirectionType.FWD, motor_power, vex.VelocityUnits.PCT)
        additional_motor1.spin(vex.DirectionType.FWD, motor_power, vex.VelocityUnits.PCT)
        additional_motor2.spin(vex.DirectionType.FWD, motor_power, vex.VelocityUnits.PCT)

        # Break the loop if the error is within an acceptable range (e.g., ±10 mm)
        if abs(error) < 10:
            break

        # Wait for the loop time before the next iteration
        vex.sleep(loop_time * 1000)

    # Stop motors after reaching the target
    left_motor.stop()
    right_motor.stop()
    additional_motor1.stop()
    additional_motor2.stop()

# Call the autonomous function during the autonomous period
autonomous()