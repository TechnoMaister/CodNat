package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDController {

    private ElapsedTime timer = new ElapsedTime();
    private double targetAngle;
    private double kP,kI, kD;
    private double integer = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public TurnPIDController(double target, double p, double i, double d){
        targetAngle = target;
        kP = p;
        kI = i;
        kD = d;
    }

    public double update(double currentAngle){
        //p
        double error = targetAngle - currentAngle;
        error %=360;
        error +=360;
        error %=360;
        if(error > 180) {
            error -= 360;
        }
        //i
        integer += error;
        if(Math.abs(error)<1) {
            integer = 0;
        }
        integer = Math.abs(integer) * Math.signum(error);

        //d
        double derivative = 0;
        if(lastTime>0) {
            derivative = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //motor power calcul
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(kP * error + kI * integer + kD * derivative);

        return motorPower;
    }
}