// *WHATEVER YOU DO, DO NOT EDIT ANYTHING HERE* //

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class encoderTestingFinal extends LinearOpMode {

    //Set motor variables
    public DcMotor motorFL;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorBR;

    //Initializing encoder positions
    public int leftPos1;
    public int leftPos2;
    public int rightPos1;
    public int rightPos2;

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

    public void runOpMode() {

        //Initialize motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //set mode to stop and reset encoders -- resets encoders to the 0 position
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize the positions to zero, since the motor has not moved yet
        leftPos1 = 0;
        leftPos2 = 0;
        rightPos1 = 0;
        rightPos2 = 0;

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //any code after this command will not be executed until the match has started
        waitForStart();

        //can now set drive distance because of the function below; now we just need to input the distance
        //can also control the direction using the mecanum drivetrain directions here: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

        //ex: this command will get the robot to travel forward (all target values are positive) for 10cm at a speed of 2
        drive(3000, 3000, 3000, 3000, 1);

        //ex: this command will get the robot to strafe left for 9cm at a speed of 1
        drive(-9, 9, 9, -9, 1);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            telemetry.addData("motorFL Encoder Position: ",motorFL.getCurrentPosition());
            telemetry.addData("motorBL Encoder Position: ",motorBL.getCurrentPosition());
            telemetry.addData("motorFR Encoder Position: ",motorFR.getCurrentPosition());
            telemetry.addData("motorBR Encoder Position: ",motorBR.getCurrentPosition());
            telemetry.update();
        }
    }

    //will use a function that will take the distance and speed of the motors based on the rotation
    //void because no return value
    public void drive(int leftTarget1, int leftTarget2, int rightTarget1, int rightTarget2, double speed) {

        double forwardTicks = 52.3;
        double strafeTicks = 54.05;

        int ticks1 = (int) forwardTicks;
        int ticks2 = (int) strafeTicks;

        leftPos1 += leftTarget1; //By adding the "+=", it makes it equivalent to leftPos1 = leftPos1 + leftTarget1, therefore it will allow adding values to the position based on what the target is.
        leftPos2 += leftTarget2; //This will therefore change where the motor needs to be by the specific inputted amount
        rightPos1 += rightTarget1;
        rightPos2 += rightTarget2;

        // Using setTargetPosition and RUN_TO_POSITION, it forces motors to continue running until the encoders reach the specified target position
        // We are multiplying by forward/strafe ticks in relation to forward/strafe based on what the robot is doing.
        if ((leftPos1 >= 0) && (leftPos2 >= 0) && (rightPos1 >= 0) && (rightPos2 >= 0)) { //Forward
            motorFL.setTargetPosition((leftPos1) * ticks1);
            motorBL.setTargetPosition((leftPos2) * ticks1);
            motorFR.setTargetPosition((rightPos1) * ticks1);
            motorBR.setTargetPosition((rightPos2) * ticks1);

        } else if ((leftPos1 < 0) && (leftPos2 < 0) && (rightPos1 < 0) && (rightPos2 < 0)) { //Backwards
            motorFL.setTargetPosition((leftPos1) * ticks1);
            motorBL.setTargetPosition((leftPos2) * ticks1);
            motorFR.setTargetPosition((rightPos1) * ticks1);
            motorBR.setTargetPosition((rightPos2) * ticks1);

        } else if (((leftPos1 < 0) && (rightPos2 < 0) && (leftPos2 >= 0) && (rightPos1 >= 0)) || (((leftPos2 < 0) && (rightPos1 < 0)) && (leftPos1 >= 0) && (rightPos2 >= 0))) {
            // Strafing left or right
            motorFL.setTargetPosition((leftPos1) * ticks2);
            motorBL.setTargetPosition((leftPos2) * ticks2);
            motorFR.setTargetPosition((rightPos1) * ticks2);
            motorBR.setTargetPosition((rightPos2) * ticks2);

        } else if (((leftPos2 > 0) && (rightPos1 > 0)) || ((leftPos1 > 0) && (rightPos2 > 0))){
            //Strafing diagonally forward right or left, respectively.
            motorFL.setTargetPosition((leftPos1) * ticks2);
            motorBL.setTargetPosition((leftPos2) * ticks2);
            motorFR.setTargetPosition((rightPos1) * ticks2);
            motorBR.setTargetPosition((rightPos2) * ticks2);

        } else if (((leftPos2 < 0) && (rightPos1 < 0)) || ((leftPos1 < 0) && (rightPos2 < 0))) {
            //Strafing diagonally backward right or left, respectively.
            motorFL.setTargetPosition((leftPos1) * ticks2);
            motorBL.setTargetPosition((leftPos2) * ticks2);
            motorFR.setTargetPosition((rightPos1) * ticks2);
            motorBR.setTargetPosition((rightPos2) * ticks2);

        } else if (((leftPos1 < 0) && (rightPos2 > 0) && (leftPos2 < 0) && (rightPos1 > 0)) || (((leftPos2 > 0) && (rightPos1 < 0)) && (leftPos1 > 0) && (rightPos2 < 0))) {
            //Turning left or right, respectively.
            motorFL.setTargetPosition((leftPos1) * ticks2);
            motorBL.setTargetPosition((leftPos2) * ticks2);
            motorFR.setTargetPosition((rightPos1) * ticks2);
            motorBR.setTargetPosition((rightPos2) * ticks2);

        }

        //Encoders do not change speed automatically. Need to adjust speed ourselves
        motorFL.setPower(speed);
        motorBL.setPower(speed);
        motorFR.setPower(speed);
        motorBR.setPower(speed);
        /*motorFL.setPower(PID(leftTarget1, motorFL.getCurrentPosition()));
        motorBL.setPower(PID(leftTarget2, motorBL.getCurrentPosition()));
        motorFR.setPower(PID(rightTarget1, motorFR.getCurrentPosition()));
        motorBR.setPower(PID(rightTarget2, motorBR.getCurrentPosition()));*/

        // The code gets stuck in between the Run to Position and the speed.
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while loop to stall/delay the next command
        while(motorFL.isBusy() && motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy()) {

        }

        //Stop driving so that it can perform the next command.
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }

    public double PID(double setPosition, double currentPosition) {
        double error = setPosition - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (setPosition * Kf);
    }
}
