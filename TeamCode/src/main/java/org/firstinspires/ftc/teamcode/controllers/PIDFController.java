package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.rConstants;

public class PIDFController {
    private double kS, kF, kP;





    public PIDFController(double kS, double kF, double kP){
        this.kS = kS;
        this.kF = kF;
        this.kP = kP;
    }






    public double calculate(double currentVoltage, double currentVelocity, double targetVelocity, double voltageCompensation, ElapsedTime timer){
        // Voltage Scaling
        double voltageRatio = currentVoltage / rConstants.ShooterConstants.nominalVoltage;
        double maxAchievableVelocity = rConstants.ShooterConstants.maximumFlyWheelVelocity / voltageRatio;
        double clampedTargetVelocity = Math.min(targetVelocity, maxAchievableVelocity);

        double error = clampedTargetVelocity - currentVelocity;


        double PIDOutput = (kP * error);
        double feedforward = 0.0;
        if (clampedTargetVelocity > 0) { feedforward = kS + (clampedTargetVelocity * kF); }

        double totalOutput = (PIDOutput + feedforward) * voltageCompensation;
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        timer.reset();

        return totalOutput;
    }





    public void setGains(double kS, double kF, double kP) {
        this.kS = kS;
        this.kF = kF;
        this.kP = kP;
    }
}
