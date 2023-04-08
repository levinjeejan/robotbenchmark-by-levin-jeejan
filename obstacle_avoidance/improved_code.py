from controller import Robot, Compass
import math

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

# Get robot's Compass device and enable it.
compass = robot.getCompass("compass")
compass.enable(timeStep)

# Get initial heading angle of the robot.
initialHeading = 0.0

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    leftVelocity = initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2
    rightVelocity = initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue

    # Check if any obstacle is detected, and make the robot turn left or right to avoid it.
    if centralSensorValue > 0.5:
        leftVelocity = -initialVelocity
        rightVelocity = initialVelocity
    elif centralLeftSensorValue > 0.3:
        leftVelocity = -initialVelocity
        rightVelocity = initialVelocity
    elif centralRightSensorValue > 0.3:
        leftVelocity = initialVelocity
        rightVelocity = -initialVelocity

    # Set the motor velocities.
    leftMotor.setVelocity(leftVelocity)
    rightMotor.setVelocity(rightVelocity)

    # Update the initial heading angle of the robot.
    if robot.getTime() < 10 * timeStep:
        initialHeading = compass.getValues()[0]

    # Calculate the current heading angle of the robot.
    currentHeading = compass.getValues()[0]

    # Adjust the turning angle based on the difference between the current heading and the initial heading.
    turnAngle = (initialHeading - currentHeading) * 0.5
    leftMotor.setVelocity(leftVelocity + turnAngle