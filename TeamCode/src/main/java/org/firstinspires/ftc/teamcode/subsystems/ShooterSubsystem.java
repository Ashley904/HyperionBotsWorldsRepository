package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardwareMap robot;
    private final PIDFController pidfController;
    private final ElapsedTime timer;

    private double targetVelocity = 0.0;




    private double cachedLeftFlyWheelMotorVelocity;
    private double cachedRightRightFlyWheelMotorVelocity;
    private double cachedVoltage;







    public ShooterSubsystem(RobotHardwareMap robotHardwareMap) {
        this.robot = robotHardwareMap;

        pidfController = new PIDFController(
                rConstants.ShooterConstants.flyWheelKs,
                rConstants.ShooterConstants.flyWheelKf,
                rConstants.ShooterConstants.flyWheelKp
        );

        timer = new ElapsedTime();
    }






    @Override
    public void periodic() {
        cachedLeftFlyWheelMotorVelocity = robot.leftFlyWheelMotor.getVelocity();
        cachedRightRightFlyWheelMotorVelocity = robot.rightFlyWheelMotor.getVelocity();
        try { cachedVoltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage(); }
        catch (Exception e) { cachedVoltage = rConstants.ShooterConstants.nominalVoltage; }


        pidfController.setGains(rConstants.ShooterConstants.flyWheelKs, rConstants.ShooterConstants.flyWheelKf, rConstants.ShooterConstants.flyWheelKp);

        if (targetVelocity > 0) {
            double output = pidfController.calculate(
                    getCurrentVoltage(),
                    getCurrentFlyWheelVelocity(),
                    targetVelocity,
                    getVoltageCompensation(),
                    timer
            );
            robot.leftFlyWheelMotor.setPower(output);
            robot.rightFlyWheelMotor.setPower(output);
        } else { robot.leftFlyWheelMotor.setPower(0); robot.rightFlyWheelMotor.setPower(0); }

        UpdateShooterState();
    }






    private void UpdateShooterState() {
        double flyWheelError = Math.abs(targetVelocity - getCurrentFlyWheelVelocity());
        if (targetVelocity <= 0) {
            rConstants.Enums.currentShooterState = rConstants.Enums.ShooterState.Disabled;
            return;
        }

        if (flyWheelError <= rConstants.ShooterConstants.velocityReachedTolerance) { rConstants.Enums.currentShooterState = rConstants.Enums.ShooterState.TargetReached; }
        else { rConstants.Enums.currentShooterState = rConstants.Enums.ShooterState.Accelerating; }
    }






    // Helpers
    public double getCurrentFlyWheelVelocity() { return (cachedLeftFlyWheelMotorVelocity + cachedRightRightFlyWheelMotorVelocity) / 2.0; }
    private double getVoltageCompensation() { return rConstants.ShooterConstants.nominalVoltage / cachedVoltage; }
    private double getCurrentVoltage() { return cachedVoltage; }


    public void setTargetVelocity(double velocity) { targetVelocity = velocity; }
    public void setHoodPosition(double position) { robot.hoodServo.setPosition(position); }
}