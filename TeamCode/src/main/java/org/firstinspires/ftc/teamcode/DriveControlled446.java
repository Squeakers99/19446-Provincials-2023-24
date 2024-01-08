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
public class DriveControlled446 extends LinearOpMode {

    //Primary Motor Defintions
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    //Secondary Motor Definitions
    private DcMotor intakeMotor;
    private DcMotor sliderMotor;
    private DcMotor liftLeft;
    private DcMotor liftRight;

    //Servo Definitions
    private Servo arm;
    private Servo wrist;
    private CRServo outtake;

    @Override
    public void runOpMode() {

        //Drivetrain DC motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Secondary system DC motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

        //Reverse left side motors and slider Motor
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoder Setup
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo Mapping
        arm = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        outtake = hardwareMap.get(CRServo.class, "outtake");

        //Arm Variables
        boolean placementReady = false;
        double armPos;
        double wristPos;
        double intakeReady_Wrist = 0.0;
        double intakeReady_Arm = 0.0;
        double placementReady_Wrist = 0.5;
        double placementReady_Arm = 0.5;

        //Slider Positioning
        double sliderPos;
        double sliderMax;
        double sliderPower;
        sliderMax = 10000;

        //Boolean variables
        boolean intakeOn = false;

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            // Gamepad inputs
            double y = -gamepad1.left_stick_y; // Reverse the y-axis (if needed)
            double x = gamepad1.right_stick_x * 1.1; //Counteracts imperfect strafing
            double rotation = (gamepad1.left_stick_x)*0.75;

            // Calculate motor powers
            double frontLeftPower = y + x + rotation;
            double frontRightPower = y - x - rotation;
            double backLeftPower = y - x + rotation;
            double backRightPower = y + x - rotation;

            // Clip motor powers to ensure they are within the valid range [-1, 1]
            frontLeftPower = Range.clip(frontLeftPower, -1, 1);
            frontRightPower = Range.clip(frontRightPower, -1, 1);
            backLeftPower = Range.clip(backLeftPower, -1, 1);
            backRightPower = Range.clip(backRightPower, -1, 1);

            // Set motor powers
            motorFL.setPower(frontLeftPower);
            motorFR.setPower(frontRightPower);
            motorBL.setPower(backLeftPower);
            motorBR.setPower(backRightPower);

            //Intake Motor Code
            if ((gamepad2.right_trigger > 0.0) && !intakeOn){
                intakeMotor.setPower(0.6);
            }else if((gamepad2.right_trigger == 0.0) && !intakeOn){
                intakeMotor.setPower(0);
            }

            if ((gamepad2.left_trigger > 0.0) && !intakeOn){
                intakeMotor.setPower(-0.6);
                outtake.setPower(-1);
            }else if((gamepad2.left_trigger == 0.0) && !intakeOn){
                intakeMotor.setPower(0);
                outtake.setPower(0);
            }

            //Arm Code
            if(gamepad2.dpad_up && !placementReady){
                slider.setTargetPosition(1000);
                arm.setPosition(placementReady_Arm);
                wrist.setPosition(placementReady_Wrist);
                placementReady = true;
            }else if(gamepad2.dpad_down && placementReady){
                slider.setTargetPosition(1000);
                arm.setPosition(intakeReady_Arm);
                wrist.setPosition(placementReady_Wrist);
            }

            //Encoder Values for the lift
            if(gamepad1.left_trigger == 1){
                liftLeft.setTargetPosition(30000);
                liftRight.setTargetPosition(30000);
                liftLeft.setPower(1);
                liftRight.setPower(1);
            }else if(gamepad1.right_trigger == 1){
                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                liftLeft.setPower(1);
                liftRight.setPower(1);
            }

            //Pixel Release
            if(gamepad2.right_bumper){
                outtake.setPower(-1);
            }else if(gamepad2.left_bumper){
                outtake.setPower(-1);
            }

            //Slider Control
            sliderPower = -gamepad2.right_stick_y;
            sliderPos = sliderMotor.getCurrentPosition();

            if(sliderPos >= 0){
                sliderMotor.setPower(sliderPower);
            }/*else if(sliderPower < 0 && sliderPos < sliderMax){
                sliderMotor.setPower(sliderPower);
            }*/else{
                sliderMotor.setPower(0);
            }

            // Drivetrain Telemetry
            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());

            //Intake Motor telemetry
            telemetry.addData("Intake Motor Power: ", intakeMotor.getPower());

            //Slider telemetry
            telemetry.addData("Slider Power: ", sliderMotor.getPower());
            telemetry.addData("Slider Position: ", sliderMotor.getCurrentPosition());

            //Lift telemetry
            telemetry.addData("Lift Left Power:", liftLeft.getPower());
            telemetry.addData("Lift Right Power:", liftRight.getPower());
            telemetry.addData("Lift Left Position:", liftLeft.getCurrentPosition());
            telemetry.addData("Lift Right Position:", liftRight.getCurrentPosition());

            //Arm and Outtake Telemetry
            telemetry.addData("Outtake Power: ", outtake.getPower());
            telemetry.addData("Arm Position: ", arm.getPosition());
            telemetry.addData("Wrist Position: ", wrist.getPosition());
            telemetry.update();
        }
    }
}
