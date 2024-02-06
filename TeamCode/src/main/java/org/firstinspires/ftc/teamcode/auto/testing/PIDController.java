package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    public ElapsedTime timer = new ElapsedTime();
    public double prevError = 0;

    public double pid(double currentPos, double desiredPos, double Kp, double Ki, double Kd) {
        double totalError = Math.abs(desiredPos - currentPos);

        //proportional term = Kp*e(t)
        double propTerm = Kp * totalError;

        //integral term
        double integral = 0;
        integral += (totalError * timer.seconds());

        //derivative term
        double derivative = Kd * (totalError/timer.seconds());

        double output = propTerm + (Ki * integral) + derivative;

        prevError = totalError;
        timer.reset();

        return output;
    }

}
