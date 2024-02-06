package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueTeleopPreset extends LinearOpMode {

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
    private Servo frontIntake1;
    private Servo frontIntake2;
    private Servo flipper;
    private CRServo outtake;

    private final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        //For some reason, reverse one of the rights (weird exception):
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        //Secondary system DC motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

        //Reverse left side motors and slider Motor
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        //For some reason, reverse one of the rights (weird exception):
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoder Setup
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo Mapping
        frontIntake1 = hardwareMap.get(Servo.class, "frontIntake1");
        frontIntake2 = hardwareMap.get(Servo.class, "frontIntake2");
        //flipper = hardwareMap.get(Servo.class, "flipper");
        //outtake = hardwareMap.get(CRServo.class, "outtake");

        //Intake Linkage Servo
        double linkageLeftPos;
        double linkageRightPos;

        double linkageLeftMax = 0.62;
        double linkageLeftMin = 0.5;
        double linkageRightMax = 0.45;
        double linkageRightMin = 0.30;

        //Flipper Variables
        boolean isFlipperOpen = false;
        double flipperPos;
        double open = 2.1;
        double closed = 0.0;

        //Slider Positioning
        double sliderPos;
        double sliderMax;
        double sliderPower;
        sliderMax = 10000;

        //Initial Positions
        linkageLeftPos = linkageLeftMin;
        linkageRightPos = linkageRightMax;

        //Boolean variables
        boolean intakeOn = false;

        //At the end of auto, should be in the parked position. Set the startPose based on that
        Pose2d StartPose = new Pose2d(-58.65, 65, Math.toRadians(0));

        drive.setPoseEstimate(StartPose);

        TrajectorySequence normalizePosition = drive.trajectorySequenceBuilder(StartPose)
                .back(20)
                .lineToLinearHeading(new Pose2d(-35.25, 48, Math.toRadians(0)))
                .build();

        //Pose is at the middle of the backdrop
        TrajectorySequence pixels = drive.trajectorySequenceBuilder(normalizePosition.end())
                .lineToLinearHeading(new Pose2d(-35.25, -10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-35.25, -53, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-35.25, -58),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .build();

        TrajectorySequence backboard = drive.trajectorySequenceBuilder(pixels.end())
                .lineToConstantHeading(new Vector2d(-35.25, -48), //inch back a bit
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .lineToLinearHeading(new Pose2d(-35.25, -15, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-35.25, 48, Math.toRadians(0)))
                .build();

        waitForStart();

        if (isStopRequested())
            return;

        //Normalize the position first:
        drive.followTrajectorySequenceAsync(normalizePosition);

        while (opModeIsActive()) {

            // Gamepad inputs
            double y = -gamepad1.left_stick_y; // Reverse the y-axis (if needed)
            double x = gamepad1.right_stick_x * 1.1; //Counteracts imperfect strafing
            double rotation = gamepad1.left_stick_x;

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
                intakeMotor.setPower(1);
            }else if((gamepad2.right_trigger == 0.0) && !intakeOn){
                intakeMotor.setPower(0);
            }

            if ((gamepad2.left_trigger > 0.0) && !intakeOn){
                intakeMotor.setPower(-1);
            }else if((gamepad2.left_trigger == 0.0) && !intakeOn){
                intakeMotor.setPower(0);
            }

            if(gamepad2.a && !intakeOn){
                intakeMotor.setPower(1);
                //outtake.setPower(1);
                intakeOn = true;
            }else if(gamepad2.a && intakeOn){
                intakeMotor.setPower(0);
                //outtake.setPower(0);
                intakeOn = false;
            }

            //Linkage Code
            linkageLeftPos += 0.05 * -gamepad2.left_stick_y;
            linkageRightPos -= 0.05 * -gamepad2.left_stick_y;

            //Set Max and Min for the linkage positions
            if (linkageLeftPos > linkageLeftMax) {
                linkageLeftPos = linkageLeftMax;
            }else if (linkageLeftPos < linkageLeftMin) {
                linkageLeftPos = linkageLeftMin;
            }else if (linkageRightPos > linkageRightMax) {
                linkageRightPos = linkageRightMax;
            }else if (linkageRightPos < linkageRightMin) {
                linkageRightPos = linkageRightMin;
            }

            //Sets Linkage Position
            frontIntake1.setPosition(linkageLeftPos);
            frontIntake2.setPosition(linkageRightPos);

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

            //Flipper control
            if(sliderPower >  0){
                flipperPos = open;
                isFlipperOpen = true;
            }else if(sliderPower < 0){
                flipperPos = closed;
                isFlipperOpen = false;
            }

            //Semi-auto teleop:
            if (gamepad1.x) { //To go to the pixels:
                drive.followTrajectorySequenceAsync(pixels);
            } else if (gamepad1.b) {
                drive.followTrajectorySequenceAsync(backboard);
            } else if (gamepad1.a) {
                stopTrajectory();
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

            //Intake Servo telemetry
            telemetry.addData("Intake Left Position: ", frontIntake1.getPosition());
            telemetry.addData("Intake Right Position: ", frontIntake2.getPosition());

            //Outtake telemetry
            //telemetry.addData("Outtake Power: ", outtake.getPower());
            //telemetry.addData("Flipper position: ", flipper.getPosition());
            telemetry.update();
        }
    }

    public void stopTrajectory() {
        drive.leftFront.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightRear.setPower(0);
    }
}
