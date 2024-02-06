package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class sliderTesting extends LinearOpMode {

    DcMotor slider;
    @Override
    public void runOpMode() {
        //hardware mapping
        slider = hardwareMap.get(DcMotor.class, "sliderMotor");

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                double power = slider.getPower();
                while (power <= 1) {
                    slider.setPower(power+0.1);
                    power += 0.1;
                    telemetry.addData("Slider Position: ", slider.getCurrentPosition());
                }
            }
        }

    }

}
