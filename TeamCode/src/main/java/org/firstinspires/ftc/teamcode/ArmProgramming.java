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

    @Override
    public void runOpMode() {
        //variables
        arm = hardwareMap.get(Servo.class, "arm");
        double armPos = 0.0;
        
        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            while(armPos <= 1.0){
                arm.setPosition(armPos);
                telemetry.addData("Arm Position: ", armPos);
                sleep(5000);
                armPos += 0.5;
            }
        }
    }
}
