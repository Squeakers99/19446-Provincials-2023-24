package org.firstinspires.ftc.teamcode.auto.pid;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//Currently in C++ and has 6 motor, change to 4 motor, java)
//For coding the auto, use encoder ticks and test
//also after each action, print sensor values (using a function like "print_sensor_values();")
//Front two wheel encoders for straight drive

/*void pidStraightFast(double target_encoder, double timeout, double kMaxSpeed, double heading) {
  // Constants for PID control
  const double kPDrive = 0.68;
  const double kIDrive = 0.005; //0.0025
  const double kDDrive = 0.0;
  const double Kpturn_correction = 2.5;
  const double move_error = 1.5;
  int end_count = 0; //number of times PID is run in terms that the robot will run PID, then check if actual angle is reached before code ends
  // Variables for tracking progress
  double startTime = pros::millis(); //timeout function


  //End count means that PID is run twice, first time is used to try to achieve 0 error, but the error that is accumalated is saved as "Previous Error" so, during the second end_count it will check how off the first PID function off and change it accordingly.
  //This two step PID is important for the code since if there is only one step, the robot will not change it's PID values to be for a smaller percent of error (it's close to the goal already just off by a few percent) Having a second end count uses the accumalation and makes sure the result is accurate.
  //Timeout function is important for the PID since the I value and overall PID function is not always able to achieve the predicted value in the time (oscillation may be in an infinite cycle since rate of change is not taken into consideration (d value))
  //comparing error and previous error this lowers the error target of the second step of PID (second endcount)


  // Variables for PID control
  double error_drive = target_encoder;
  double integral_drive = 0.0;
  double derivative_drive = 0.0;
  double prevError_drive = 0.0; //subs in first end_count error (how much it was off in the first time) based off how it has a different error, this means the PID will change

  // Reset the drive motor encoders
  encoderLEFT.reset();
  encoderRIGHT.reset();

  // Main PID control loop
  while (end_count < 2 && pros::millis() - startTime < timeout) {
    // Get the current encoder counts for the left and right drive motors
    double leftEncoderValue = encoderLEFT.get_value();
    double rightEncoderValue = encoderRIGHT.get_value();
    if (std::abs(error_drive) < move_error) {
      end_count += 1;
    }
    else{
      end_count = 0;
    }

    // Calculate the distance travelled by the robot
    double current_dist = (leftEncoderValue + rightEncoderValue) / 2.0 ;

    // Update the total distance travelled
    //totalDistance = distanceTravelled;

    // Print the total distance travelled to the screen
    pros::lcd::print(6, "Total Distance: %4.1f\n", current_dist);

    // Calculate the error, integral, and derivative terms for PID control
    prevError_drive = error_drive;
    error_drive = target_encoder - current_dist;
    if (prevError_drive*error_drive < 0) {
      integral_drive = 0;
    }
    if (std::abs(error_drive) < 50) {
    integral_drive += error_drive;
    }
    derivative_drive = error_drive - prevError_drive;

    // Calculate the motor speeds using PID control
    double leftSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive;
    double rightSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive;
    // Limit the motor speeds to be within the acceptable range

    if (leftSpeed < 0 )
      leftSpeed = std::max(leftSpeed, -kMaxSpeed);
    else
      leftSpeed = std::min(leftSpeed, kMaxSpeed);
    if (rightSpeed < 0 )
      rightSpeed = std::max(rightSpeed, -kMaxSpeed);
    else
      rightSpeed = std::min(rightSpeed, kMaxSpeed);

    leftSpeed += Kpturn_correction *(heading - imu.get_yaw());
    rightSpeed -= Kpturn_correction *(heading - imu.get_yaw());

    // Set the motor speeds
    lfm.move_velocity(leftSpeed);
    lmm.move_velocity(leftSpeed);
    lbm.move_velocity(leftSpeed);
    rfm.move_velocity(rightSpeed);
    rmm.move_velocity(rightSpeed);
    rbm.move_velocity(rightSpeed);
  // Wait for the motors to update
    pros::delay(20);
  }

  // Stop the motors once the target distance has been reached
  lfm.move_velocity(0);
  lmm.move_velocity(0);
  lbm.move_velocity(0);
  rfm.move_velocity(0);
  rmm.move_velocity(0);
  rbm.move_velocity(0);
  }

void pidTurn(double targetAngle, double timeout) {
  // Reset the PID variables
  double error = targetAngle - imu.get_yaw();
  double prevError = 0.0;
  double integral = 0.0;
  double derivative = 0.0;
  double output = 0.0;

  // Define constants for PID algorithm
  // const double Kp = 0.7; //1.1 max
  // const double Ki = 0.01; //0.0025
  // const double Kd = 0.0;
  double Kp = 0.0012*error + 2.39; //average error of the KP in the robot multiplied by the percent in thousandths plus the KP maximizes KP value/perfects the KP value
  //const double Kp = 2.58; //150 degrees: 2.58 (a bit over), 50: 2.45  20:2.42
  const double Ki = 0.02; //150 degrees: 0.02
  const double Kd = 0.0; //KD is ineffective in turning since it creates oscillation whereas in turning it is better to perfect KP and KI so that it slightly overshoots and uses the I value to correct
  const double minspeed = 10;
  double startTime = pros::millis();
  int end_count = 0;

  //imu.set_yaw(0);
  // Loop until robot reaches target angle
  while (end_count < 2 || std::abs(error) > std::abs(prevError)) {
    // Update prevError
    prevError = error;
    // Calculate error
    double elapsedTime = pros::millis() - startTime;
        if (elapsedTime > timeout) {
            break;
        }

    if (std::abs(error) < 0.7) {
      end_count += 1;
    }
    else{
      end_count = 0;
    }
    error = targetAngle - imu.get_yaw();
    pros::lcd::set_text(7, "IMU Yaw: " + std::to_string(imu.get_yaw()));
    // Calculate the proportional component of PID algorithm
    double proportional = Kp * error;

    // Calculate the integral component of PID algorithm
    if (prevError*error < 0) {
      integral = 0;
    }
    // Calculate the derivative component of PID algorithm
    derivative = (error - prevError);
    if (std::abs(output) < 45) {
    integral += Ki * error;
    }
    // Calculate the output of the PID algorithm
    output = proportional + integral + derivative*Kd;
    pros::lcd::set_text(0, "Output: " + std::to_string(output));
    // Set the motor speeds based on the output
    lfm.move_velocity(output);
    lmm.move_velocity(output);
    lbm.move_velocity(output);
    rfm.move_velocity(-output);
    rmm.move_velocity(-output);
    rbm.move_velocity(-output);



    // Wait a short time before looping again
    pros::delay(20);
  }

  // Stop the motors when target is reached
  lfm.move(0);
  lmm.move(0);
  lbm.move(0);
  rfm.move(0);
  rmm.move(0);
  rbm.move(0);
} */

@Autonomous
public class TurnAuto extends LinearOpMode {
    public DcMotor motorFL;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorBR;
    public BNO055IMU imu;

    //Timer
    ElapsedTime timer = new ElapsedTime();

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

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //any code after this command will not be executed until the match has started
        waitForStart();

        telemetry.addData("Original IMU: ", getInitialHeading());
        telemetry.update();

        goToHeading(90, 5000);
        sleep(5000);

        telemetry.addData("New IMU: ", imu.getAngularOrientation());
        telemetry.update();

        sleep(15000);

        while (opModeIsActive()) {
            print_sensor_values();
        }

    }

    //Returns imu angle
    private double getInitialHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

//    private void goToHeading(double targetAngle, double timeout) {
//        // double currentHeading = imu.getAngularOrientation().firstAngle;
//        // double error = targetHeading - currentHeading;
//        // double kP = 0.01;
//        // double correction = kP * error;
//        // int end_count = 0;
//        //targetAngle = 0;
//        // // Apply correction to the motors
//        // motorFL.setPower(-correction);
//        // motorBL.setPower(-correction);
//        // motorFR.setPower(correction);
//        // motorBR.setPower(correction);
//
//        // // Continue adjusting until the error is small enough
//        // //while (end_count < 2 || std::abs(error) > std::abs(prevError)) {
//        // w=
//        // while (opModeIsActive() && Math.abs(error) > 2.0) {
//        //     currentHeading = imu.getAngularOrientation().firstAngle;
//        //     error = targetHeading - currentHeading;
//
//        //     correction = kP * error;
//
//        //     motorFL.setPower(-correction);
//        //     motorBL.setPower(-correction);
//        //     motorFR.setPower(correction);
//        //     motorBR.setPower(correction);
//
//        //     idle();
//        // Reset the PID variables
//
//        //double error = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        double error = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
//
//        telemetry.addData("Error: ", error);
//        telemetry.update();
//        sleep(7000);
//
//        double prevError = 0.0;
//        double integral = 0.0;
//        double derivative = 0.0;
//        double output = 0.0;
//
//        // Define constants for PID algorithm
//        // const double Kp = 0.7; //1.1 max
//        // const double Ki = 0.01; //0.0025
//        // const double Kd = 0.0;
//        double Kp = 0.05;
//        //double Kp = 0.001 * error + 0.03;
//        //const double Kp = 2.58; //150 degrees: 2.58 (a bit over), 50: 2.45  20:2.42
//        double Ki = 0.0; //150 degrees: 0.02
//        double Kd = 0.0;
//        double startTime = timer.milliseconds();
//
//        int end_count = 0;
//
//        //imu.set_yaw(0);
//        // Loop until robot reaches target angle
//        while (end_count < 2 || Math.abs(error) > Math.abs(prevError)) {
//            // Update prevError
//            prevError = error;
//
//            // Calculate error
//            double elapsedTime = timer.milliseconds() - startTime;
//            if (elapsedTime > timeout) {
//                break;
//            }
//
//            //Start to stop oscillation of bot
//            //if (Math.abs(error) < 0.7) {
//            if (Math.abs(error) < 1.0) {
//                end_count += 1;
//            } else {
//                end_count = 0;
//            }
//
//        error = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
//        // change to telemetry
//        //pros::lcd::set_text(7, "IMU Yaw: " + std::to_string(imu.get_yaw()));
//        //telemetry.addData("IMU Yaw: ", String.valueOf(imu.getAngularOrientation()));
//
//        // Calculate the proportional component of PID algorithm
//        double proportional = Kp * error;
//
//        // Calculate the integral component of PID algorithm
//        if (prevError * error < 0) {
//            integral = 0;
//        }
//        // Calculate the derivative component of PID algorithm
//        derivative = (error - prevError);
//        if (Math.abs(output) < 45) {
//            integral += Ki * error;
//        }
//        // Calculate the output of the PID algorithm
//        output = proportional + integral + derivative * Kd;
//        // change to telemetry _>
//        //pros::lcd::set_text(0, "Output: " + std::to_string(output));
//        //telemetry.addData("Output: ", String.valueOf(output));
//
//        // Set the motor speeds based on the output
//        motorFL.setPower(-output);
//        motorBL.setPower(-output);
//        motorFR.setPower(output);
//        motorBR.setPower(output);
//        // Wait a short time before looping again
//        sleep(20);
//    }
//        telemetry.addData("IMU Yaw: ", String.valueOf(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)));
//        telemetry.addData("Output: ", String.valueOf(output));
//        telemetry.addData("Error (new): ", error);
//        telemetry.update();
//
//    // Stop the motors after reaching the target heading
//        motorFL.setPower(0);
//        motorBL.setPower(0);
//        motorFR.setPower(0);
//        motorBR.setPower(0);
//}

    private void goToHeading(double targetAngle, double timeout) {
        double error = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

        double prevError = 0.0;
        double integral = 0.0;
        double derivative;
        double output;

        double Kp = 1.0;
        double Ki = 0.0;
        double Kd = 0.0;

        double startTime = timer.milliseconds();

        while (opModeIsActive()) {
            double elapsedTime = timer.milliseconds() - startTime;

            if (elapsedTime > timeout) {
                break;
            }

            prevError = error;
            error = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

            integral += Ki * error;
            if (Math.abs(integral) > 0.1) {  // Implement an integral limit
                integral = Math.signum(integral) * 0.1;
            }

            derivative = error - prevError;

            output = Kp * error + integral + Kd * derivative;

            // Limit motor power values
            output = Math.max(-1.0, Math.min(1.0, output));

            motorFL.setPower(-output);
            motorBL.setPower(-output);
            motorFR.setPower(output);
            motorBR.setPower(output);

            if (Math.abs(error) < 5) {  // Adjust this threshold based on your requirements
                break;
            }

            sleep(20);
        }

        // Stop the motors after reaching the target heading
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        telemetry.addData("Final IMU Yaw: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        telemetry.update();
    }

    void print_sensor_values() {
        telemetry.addData("IMU Yaw: ", String.valueOf(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)));
        telemetry.addData("IMU Accel X: ", String.valueOf(imu.getAngularVelocity()));
        telemetry.update();
    }

    void print_travel_distance() {
        double ticksPerInch = 360.0 / (3.14159 * 3.75);
        double totalDistance = 0.0;

        // // Get the current encoder counts for the left and right drive motors
        // double leftEncoderValue = encoderLEFT.get_value();
        // double rightEncoderValue = encoderRIGHT.get_value();
        // // Calculate the distance travelled by the robot
        // //double distanceTravelled = (leftEncoderValue + rightEncoderValue) / (2.0 * ticksPerInch);

        // // Update the total distance travelled
        // //totalDistance = distanceTravelled;

        // // Print the total distance travelled to the screen
        // pros::lcd::print(6, "Total Distance: %4.1f inches\n", totalDistance);

        //Front two motors:
        double leftEncoderValue = motorFL.getCurrentPosition();
        double rightEncoderValue = motorFR.getCurrentPosition();

        double distanceTravelled = (leftEncoderValue + rightEncoderValue) / (2.0 * ticksPerInch);
        totalDistance = distanceTravelled;

        telemetry.addData("Total Distance: ", totalDistance);
    }

}