// FINALIZED DRIVE CONTROLLED, DO NOT TOUCH ANYTHING HERE //

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class driveControlled extends LinearOpMode {
    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            //Driving

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            //STRAFING VARIABLE
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            //THIS IS THE TURNING VARIABLE
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            if (gamepad1.dpad_up) {
                motorFL.setPower(1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(1);
            }

            if (gamepad1.dpad_down) {
                motorFL.setPower(-1);
                motorBL.setPower(-1);
                motorFR.setPower(-1);
                motorBR.setPower(-1);
            }

            if (gamepad1.dpad_left) {
                motorFL.setPower(-1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(-1);
            }

            if (gamepad1.dpad_right) {
                motorFL.setPower(1);
                motorBL.setPower(-1);
                motorFR.setPower(-1);
                motorBR.setPower(1);
            }

            //Change buttons later
            //y & x mean y-coordinates (up and down) and x-coordinates (left and right), respectively.
            //Manual Lift (apparently two lifts?)

        /*    //Lift Presets
            if (gamepad2.dpad_down) {
                while ((Touch1.getState() == true) && (Touch2.getState() == true)) {
                    telemetry.addData("Touch Sensors", "Are Not Pressed");
                    Lift1.setPower(1);
                    Lift2.setPower(1);

                }
                while ((Touch1.getState() == false) && (Touch2.getState() == false)) {
                    telemetry.addData("Touch Sensors", "Are Now Pressed");
                    Lift1.setPower(0);
                    Lift2.setPower(0);
                }
            }
         */

            //Roller Flipper
         /*   flipperMotorPower = gamepad1.touchpad_finger_1_y;
            flipperMotorPower = Range.clip(flipperMotorPower, -1, 1);
            rollerFlipper.setPower(flipperMotorPower);
*/
            //don't need | rollerFlipper2Power = gamepad1.right_stick_y;
            //don't need | rollerFlipper2Power = Range.clip(rollerFlipper2Power, -1, 1)
/*            Servo rollerFlipper2 = hardwareMap.get(Servo.class, "rollerFlipper 2");
            rollerFlipper2.setPosition(0);
            double rollerFlipper2Power;

 //           int clickB = 0;
            //Click b to do the roller flipper thing.
 //           if (gamepad1.b) {
            /*clickB += 1;
                if (clickB % 2 == 1) {
                   rollerFlipper2Power = -0.5; //servo 90 deg to the left
                } else {
                    rollerFlipper2Power = 0.5; //servo 90 deg to the right
                }
                rollerFlipper2.setPosition(rollerFlipper2Power);
            }
*/
            //alternatively do this?:
   /*         if (gamepad1.right_bumper) {
                rollerFlipper2.setPosition(0.25);
                rollerFlipper.setPower(1);
            }


       if (gamepad1.left_bumper) {
               rollerFlipper2.setPosition(-0.25);
                rollerFlipper.setPower(1);
           }
*/
            //Intake
            /*
            intakeMotorPower = gamepad1.touchpad_finger_1_y;
            intakeMotorPower = Range.clip(intakeMotorPower, -1, 1);
            Intake1.setPower(intakeMotorPower);
            Intake2.setPower(intakeMotorPower);
            */


            //cam picker-upper (idk the name lmao its the thing that goes in the hole of the cone and picks it up)
            //changes on and off via clicking the same button. if the user clicks once, it will go up, if the user clicks twice, it will go down.
            /*
            //PRESETS:
            //1cm = 52.4 ticks

            //Ground junction preset position.
            if (gamepad2.a){
                moveLift(1,500);
                verticalRack.setPosition(-1);
            }

            //Low junction preset position  original 1782 ticks
            if (gamepad2.x){  //34cm
                moveLift(1,1400);
                verticalRack.setPosition(1);
            }
            //Medium junction preset position
            if (gamepad2.b){ //59 cm
                moveLift(1,2500);
                verticalRack.setPosition(1);
            }
            //High junction preset position
            if (gamepad2.y) { //84 cm
                moveLift(1,3500);
                verticalRack.setPosition(1);
            }
            */

            //lift presents (low, med, high) have to put actual button


            // Add sensors to telemetry???
            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());
            telemetry.addData("LF Position:", motorFL.getCurrentPosition());
            telemetry.addData("LB Position:", motorBL.getCurrentPosition());
            telemetry.addData("RF Position:", motorFR.getCurrentPosition());
            telemetry.addData("RB Position:", motorBR.getCurrentPosition());

            telemetry.update();
        }
    }
}
