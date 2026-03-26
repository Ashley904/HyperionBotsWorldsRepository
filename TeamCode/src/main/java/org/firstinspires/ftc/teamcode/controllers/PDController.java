package org.firstinspires.ftc.teamcode.controllers;

public class PDController {
    private final double kP, kD;

    double previousError=0.0;
    double previousTime=0.0;





    public PDController(double kP, double kD){
        this.kP = kP;
        this.kD = kD;
    }






    public double calculate(double current, double target, double minimumCorrectionSpeed , double maximumCorrectionSpeed){
        double error = target - current;
        double timeElapsed = System.currentTimeMillis();

        double deltaError = error - previousError;
        double deltaTime = timeElapsed - previousTime;

        double derivative = (deltaTime > 0) ? deltaError / deltaTime : 0.0; // Derivative dampening

        double output = (error * kP) + (derivative * kD);

        previousError = error;
        previousTime = timeElapsed;

        output = Math.min(Math.max(output, minimumCorrectionSpeed), maximumCorrectionSpeed);
        return output;
    }
}
