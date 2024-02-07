package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class ArmProgramming extends LinearOpMode {

    //Servo Definitions
    private Servo arm;
    private Servo wrist;

    @Override
    public void runOpMode() {
        //variables
        arm = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class,"wrist");

        wrist.setDirection(Servo.Direction.REVERSE);

        double armPos = 0.0;
        double wristPos = 0.0;
        
        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            if(gamepad1.y && wristPos <= 1.0 && wristPos >= 0.0){
                wristPos += 0.05;
                sleep(1000);
            }else if(gamepad1.x && wristPos <= 1.0 && wristPos >= 0.0){
                wristPos -= 0.05;
                sleep(1000);
            }else if(gamepad1.dpad_up && armPos <= 1.0 && armPos >= 0.0){
                armPos += 0.05;
                sleep(1000);
            }else if(gamepad1.dpad_down && armPos <= 1.0 && armPos >= 0.0){
                armPos -= 0.05;
                sleep(1000);
            }

            wrist.setPosition(wristPos);
            arm.setPosition(armPos);

            telemetry.addData("Wrist Position: ", wristPos);
            telemetry.addData("Arm Position: ", armPos);
            telemetry.update();
        }
    }
}
