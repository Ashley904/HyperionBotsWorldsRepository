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
    private static final double gearRatio = 86.0 / 34.0;
    private static final double ticksPerDegree = (ticksPerRevolution * gearRatio) / 360.0;

    public TurretSubsystem(RobotHardwareMap robot){
        this.robot = robot;
        pdController = new PDController(
                rConstants.TurretConstants.turretKp,
                rConstants.TurretConstants.turretKd
        );
    }

    @Override
    public void periodic(){
        pdController.setGains(rConstants.TurretConstants.turretKp, rConstants.TurretConstants.turretKd);

        double currentTicks = robot.turretMotor.getCurrentPosition();
        double targetTicks = targetTurretAngle * ticksPerDegree;

        double error = targetTicks - currentTicks;
        double output = pdController.calculate(0, error, -rConstants.TurretConstants.maxTurretPower, rConstants.TurretConstants.maxTurretPower);

        robot.turretMotor.setPower(output);
    }

    public void setTurretAngle(double targetTurretAngle) { this.targetTurretAngle = targetTurretAngle; }

    public double getTargetAngle() { return targetTurretAngle; }
    public double getCurrentTurretPosition() { return robot.turretMotor.getCurrentPosition(); }
    public double getTargetTurretPosition() { return targetTurretAngle * ticksPerDegree; }
}