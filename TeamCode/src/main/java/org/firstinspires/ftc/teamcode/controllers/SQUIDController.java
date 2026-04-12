package org.firstinspires.ftc.teamcode.controllers;

public class SQUIDController {
    private double previousError = 0.0;
    private double previousTime = -1.0;






    public double calculate(double error, double kSQ, double kD){
        double currentTime = System.nanoTime() / 1e9;

        // Derivative Calculation
        double derivative = 0.0;
        if(previousTime >= 0.0){
            double deltaTime = currentTime - previousTime;

            if(deltaTime > 0.0) { derivative = (error - previousError) / deltaTime; }
        }

        previousError = error;
        previousTime = currentTime;

        double squidCalculation = Math.sqrt(Math.abs(error * kSQ)) * Math.signum(error);
        double derivativeOutput = derivative * kD;

        return squidCalculation + derivativeOutput;
    }
}
