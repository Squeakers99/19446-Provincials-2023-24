package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class EncoderAutonomous {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    // Define constants for motor encoder counts
    public static final int TICKS_PER_REVOLUTION = 1440; // Replace with your motor's ticks per revolution
    public static final double WHEEL_DIAMETER_INCHES = 4.0; // Replace with your wheel diameter
    public static final double DRIVE_SPEED = 0.5; // Adjust the speed as needed
    public static final int TARGET_DISTANCE_INCHES = 12; // Replace with your target distance
    public static final double RADIUS = 1.875;

    double integralSum = 0;
    double Kp = 0.05;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    public EncoderAutonomous(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB) {
        motorFrontLeft = LF;
        motorFrontRight = RF;
        motorBackLeft = LB;
        motorBackRight = RB;
    }


    public void stopMotors() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        motorFrontLeft.setMode(mode);
        motorFrontRight.setMode(mode);
        motorBackLeft.setMode(mode);
        motorBackRight.setMode(mode);

    }

    public void forward(int target) {
        // Calculate the target encoder ticks based on distance
        target = (int) (TICKS_PER_REVOLUTION * TARGET_DISTANCE_INCHES / (Math.PI * WHEEL_DIAMETER_INCHES));

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(target);
        motorFrontRight.setTargetPosition(target);
        motorBackLeft.setTargetPosition(target);
        motorBackRight.setTargetPosition(target);

        // Run to target position
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until all motors reach the target position
        while ( motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }
        // Stop all motors
        stopMotors();
        // Reset motor modes to RUN_USING_ENCODER
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void backward(int target) {
        // Calculate the target encoder ticks based on distance
        target = (int) (TICKS_PER_REVOLUTION * TARGET_DISTANCE_INCHES / (Math.PI * WHEEL_DIAMETER_INCHES));

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(-target);
        motorFrontRight.setTargetPosition(-target);
        motorBackLeft.setTargetPosition(-target);
        motorBackRight.setTargetPosition(-target);

        // Run to target position
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        motorFrontLeft.setPower(PID(-target, motorFrontLeft.getCurrentPosition()));
        motorFrontRight.setPower(PID(-target, motorFrontRight.getCurrentPosition()));
        motorBackLeft.setPower(PID(-target, motorBackLeft.getCurrentPosition()));
        motorBackRight.setPower(PID(-target, motorBackLeft.getCurrentPosition()));


        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }
        // Stop all motors
        stopMotors();
        // Reset motor modes to RUN_USING_ENCODER
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeft(int target) {
        // Calculate the target encoder ticks based on distance
        target = (int) (TICKS_PER_REVOLUTION * TARGET_DISTANCE_INCHES / (Math.PI * WHEEL_DIAMETER_INCHES));

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(-target);
        motorFrontRight.setTargetPosition(target);
        motorBackLeft.setTargetPosition(target);
        motorBackRight.setTargetPosition(-target);

        // Run to target position
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        motorFrontLeft.setPower(PID(target, motorFrontLeft.getCurrentPosition()));
        motorFrontRight.setPower(PID(target, motorFrontRight.getCurrentPosition()));
        motorBackLeft.setPower(PID(target, motorBackLeft.getCurrentPosition()));
        motorBackRight.setPower(PID(target, motorBackLeft.getCurrentPosition()));


        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }
        // Stop all motors
        stopMotors();
        // Reset motor modes to RUN_USING_ENCODER
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRight(int target) {
        // Calculate the target encoder ticks based on distance
        target = (int) (TICKS_PER_REVOLUTION * TARGET_DISTANCE_INCHES / (Math.PI * WHEEL_DIAMETER_INCHES));

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(target);
        motorFrontRight.setTargetPosition(-target);
        motorBackLeft.setTargetPosition(-target);
        motorBackRight.setTargetPosition(target);


        // Run to target position
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        motorFrontLeft.setPower(PID(target, motorFrontLeft.getCurrentPosition()));
        motorFrontRight.setPower(PID(target, motorFrontRight.getCurrentPosition()));
        motorBackLeft.setPower(PID(target, motorBackLeft.getCurrentPosition()));
        motorBackRight.setPower(PID(target, motorBackLeft.getCurrentPosition()));

        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }
        // Stop all motors
        stopMotors();
        // Reset motor modes to RUN_USING_ENCODER
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafePositiveSlope(int target) {
        // Calculate the target encoder ticks based on distance
        target = (int) (TICKS_PER_REVOLUTION * TARGET_DISTANCE_INCHES / (Math.PI * WHEEL_DIAMETER_INCHES));

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(target);
        motorBackRight.setTargetPosition(target);

        // Run to target position
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        motorFrontLeft.setPower(PID(target, motorFrontLeft.getCurrentPosition()));
        motorBackRight.setPower(PID(target, motorBackLeft.getCurrentPosition()));

        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }
        // Stop all motors
        stopMotors();
        // Reset motor modes to RUN_USING_ENCODER
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeNegativeSlope(int target) {
        // Calculate the target encoder ticks based on distance
        target = (int) (TICKS_PER_REVOLUTION * TARGET_DISTANCE_INCHES / (Math.PI * WHEEL_DIAMETER_INCHES));

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontRight.setTargetPosition(target);
        motorBackLeft.setTargetPosition(target);

        // Run to target position
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        motorFrontRight.setPower(PID(target, motorFrontRight.getCurrentPosition()));
        motorBackLeft.setPower(PID(target, motorBackLeft.getCurrentPosition()));

        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }
        // Stop all motors
        stopMotors();
        // Reset motor modes to RUN_USING_ENCODER
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnRight(int angle) {
        double distance = (angle / 360.0) * (2.0 * Math.PI * RADIUS);
        int convertedDistance = (int) distance;

        // Reset encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(convertedDistance);
        motorFrontRight.setTargetPosition(-convertedDistance);
        motorBackLeft.setTargetPosition(convertedDistance);
        motorBackRight.setTargetPosition(-convertedDistance);

        // Run to target position
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(PID(convertedDistance, motorFrontLeft.getCurrentPosition()));
        motorFrontRight.setPower(PID(convertedDistance, motorFrontRight.getCurrentPosition()));
        motorBackLeft.setPower(PID(convertedDistance, motorBackLeft.getCurrentPosition()));
        motorBackRight.setPower(PID(convertedDistance, motorBackLeft.getCurrentPosition()));

        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }

        // Stop all motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        // Reset motor modes to RUN_USING_ENCODER
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnLeft(int angle) {
        double distance = (angle / 360.0) * (2.0 * Math.PI * RADIUS);
        int convertedDistance = (int) distance;

        // Reset encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(-convertedDistance);
        motorFrontRight.setTargetPosition(convertedDistance);
        motorBackLeft.setTargetPosition(-convertedDistance);
        motorBackRight.setTargetPosition(convertedDistance);

        // Run to target position
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(PID(convertedDistance, motorFrontLeft.getCurrentPosition()));
        motorFrontRight.setPower(PID(convertedDistance, motorFrontRight.getCurrentPosition()));
        motorBackLeft.setPower(PID(convertedDistance, motorBackLeft.getCurrentPosition()));
        motorBackRight.setPower(PID(convertedDistance, motorBackLeft.getCurrentPosition()));

        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }

        // Stop all motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        // Reset motor modes to RUN_USING_ENCODER
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // reference is encoder ticks, state is current position
    public double PID(double setPosition, double currentPosition) {
        double error = setPosition - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (setPosition * Kf);
    }
}