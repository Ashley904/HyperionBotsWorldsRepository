package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.controllers.PDController;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class TurretSubsystem extends SubsystemBase {
    private final RobotHardwareMap robot;
    private final PDController pdController;

    private double targetTurretAngle;

    private static final double ticksPerRevolution = 384.5;
    private static final double gearRatio = 86.0 / 35.0;
    private static final double ticksPerRadian = (ticksPerRevolution / (2 * Math.PI)) * gearRatio;





    public TurretSubsystem(RobotHardwareMap robot){
        this.robot = robot;
        pdController = new PDController(
                rConstants.TurretConstants.turretKp,
                rConstants.TurretConstants.turretKd
        );

        // Reset encoder so "0 ticks" = wherever the turret is at init
        robot.turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Apply direction config from constants
        robot.turretMotor.setDirection(
                rConstants.TurretConstants.turretMotorInverted
                        ? com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE
                        : com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD
        );

        // Brake by default so it holds position
        robot.turretMotor.setZeroPowerBehavior(
                rConstants.TurretConstants.floatModeEnabled
                        ? com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
                        : com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
        );
    }





    @Override
    public void periodic(){
        pdController.setGains(rConstants.TurretConstants.turretKp, rConstants.TurretConstants.turretKd);

        double targetTicks = Math.toRadians(targetTurretAngle) * ticksPerRadian;

        double adjustment = pdController.calculate(
                robot.turretMotor.getCurrentPosition(),
                targetTicks,
                -rConstants.TurretConstants.maxTurretPower,
                rConstants.TurretConstants.maxTurretPower
        );

        robot.turretMotor.setPower(adjustment);
    }





    public void setTurretAngle(double targetTurretAngle) {
        this.targetTurretAngle = Math.max(
                -rConstants.TurretConstants.turretMaxLeftAngle,
                Math.min(rConstants.TurretConstants.turretMaxRightAngle, targetTurretAngle)
        );
    }
    public double getTargetAngle() { return targetTurretAngle; }
    public double getCurrentTurretPosition() { return robot.turretMotor.getCurrentPosition(); }
    public double getTargetTurretPosition() { return Math.toRadians(targetTurretAngle) * ticksPerRadian; }
}