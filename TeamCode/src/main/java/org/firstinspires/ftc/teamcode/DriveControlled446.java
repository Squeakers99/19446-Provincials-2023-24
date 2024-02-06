package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive Code",group="OpMode")
public class DriveControlled446 extends OpMode {

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

    //Arm Variables
    boolean placementReady = false;
    double armPos;
    double wristPos;
    double intakeReady_Wrist = 0;
    double intakeReady_Arm = 0;
    double placementReady_Wrist = 0.7;
    double placementReady_Arm = 0.0;

    //Slider Positioning
    double sliderPos;
    double sliderMax = 5000;
    double sliderPower;

    //Boolean variables
    boolean intakeOn = false;

    @Override
    public void init(){
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

        //Servo Mapping
        arm = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        outtake = hardwareMap.get(CRServo.class, "outtake");

        //Reverse left side motors and wrist servo
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.REVERSE);

        //Encoder Setup
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        sliderMotor.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop(){
        arm.setPosition(intakeReady_Arm);
        wrist.setPosition(intakeReady_Wrist);
    }

    @Override
    public void loop() {
        // Gamepad inputs
        double y = -gamepad1.left_stick_y; // Reverse the y-axis (if needed)
        double x = -gamepad1.right_stick_x * 1.1; //Counteracts imperfect strafing
        double rotation = gamepad1.left_stick_x*0.75;

        // Calculate motor powers
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double frontLeftPower = (y + x + rotation) / denominator;
        double backLeftPower = (y - x + rotation) / denominator;
        double frontRightPower = (y - x - rotation) / denominator;
        double backRightPower = (y + x - rotation) / denominator;

        //Sets motor powers
        motorFL.setPower(frontLeftPower);
        motorBL.setPower(backLeftPower);
        motorFR.setPower(frontRightPower);
        motorBR.setPower(backRightPower);

        //Intake Motor Code
        if (gamepad2.right_bumper){
            intakeMotor.setPower(0.5);
            outtake.setPower(-1);
            intakeOn = true;
        }else if(!gamepad2.left_bumper && !gamepad2.right_bumper){
            intakeMotor.setPower(0);
            outtake.setPower(0);
            intakeOn = false;
        }

        if (gamepad2.left_bumper){
            intakeMotor.setPower(-0.5);
            outtake.setPower(1);
            intakeOn = true;
        }else if(!gamepad2.left_bumper && !gamepad2.right_bumper){
            intakeMotor.setPower(0);
            outtake.setPower(0);
            intakeOn = false;
        }

        //Arm Code
        if(gamepad2.dpad_up && !placementReady){
            arm.setPosition(placementReady_Arm);
            wrist.setPosition(placementReady_Wrist);
            placementReady = true;
        }else if(gamepad2.dpad_down && placementReady){
            arm.setPosition(intakeReady_Arm);
            wrist.setPosition(intakeReady_Wrist);
            placementReady = false;
        }

        //Encoder Values for the lift
        if(gamepad1.right_trigger == 1){
            liftLeft.setTargetPosition(13000);
            liftRight.setTargetPosition(13000);
            liftLeft.setPower(1);
            liftRight.setPower(1);
        }else if(gamepad1.left_trigger == 1){
            liftLeft.setTargetPosition(0);
            liftRight.setTargetPosition(0);
            liftLeft.setPower(1);
            liftRight.setPower(1);
        }

        //Slider Control
        sliderPower = -gamepad2.right_stick_y;
        sliderPos = sliderMotor.getCurrentPosition();
        sliderMotor.setPower(sliderPower);

        if(gamepad2.a){
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotor.setTargetPosition(0);
            sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Drivetrain Telemetry
        telemetry.addData("LF Power:", motorFL.getPower());
        telemetry.addData("LB Power:", motorBL.getPower());
        telemetry.addData("RF Power:", motorFR.getPower());
        telemetry.addData("RB Power:", motorBR.getPower());

        telemetry.addData("LF Encoder:",motorFL.getCurrentPosition());
        telemetry.addData("LB Encoder:",motorBL.getCurrentPosition());
        telemetry.addData("RF Encoder:",motorFR.getCurrentPosition());
        telemetry.addData("RB Encoder:",motorBR.getCurrentPosition());

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

    @Override
    public void stop(){
        arm.setPosition(intakeReady_Arm);
        wrist.setPosition(intakeReady_Wrist);
    }

    private void justWait(int miliseconds){
        double currTime = getRuntime();
        double waitUntil = currTime + (double)(miliseconds/1000);
        while (getRuntime() < waitUntil){
        }
    }
}
