package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class BlueLeftSimpleAuto extends LinearOpMode {

    DcMotorEx leftFront, leftRear, rightRear, rightFront;
    @Override
    public void runOpMode() {
        //Initialize DC Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorFrontRight");

        //Reverse left motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderAutonomous robot = new EncoderAutonomous(leftFront, leftRear, rightFront, rightRear);
        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) { return; }

        robot.forward(3000);

        /*
        robot.strafeNegativeSlope(400);
        robot.turnLeft(90);
        robot.forward(500);*/

    }

}
