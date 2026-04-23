package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controllers.PDController;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class TurretSubsystem extends SubsystemBase {
    private final RobotHardwareMap robot;
    private final PDController pdController;

    private double targetTurretAngle = 0;
    private double targetTicks = 0;

    private static final double ticksPerMotorRevolution = 384.5;
    private static final double gearRatio = 86.0 / 35.0;
    private static final double ticksPerTurretRevolution = ticksPerMotorRevolution * gearRatio;
    private static final double ticksPerRadian = ticksPerTurretRevolution / (2 * Math.PI);
    private static final double ticksPerDegree = ticksPerTurretRevolution / 360.0;


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

        double power = pdController.calculate(
                robot.turretMotor.getCurrentPosition(),
                targetTicks,
                -rConstants.TurretConstants.maxTurretPower,
                rConstants.TurretConstants.maxTurretPower
        );

        robot.turretMotor.setPower(power);
    }


    public void setTurretAngle(double angleDegrees) {
        double min = Math.min(rConstants.TurretConstants.turretMinAngle, rConstants.TurretConstants.turretMaxAngle);
        double max = Math.max(rConstants.TurretConstants.turretMinAngle, rConstants.TurretConstants.turretMaxAngle);

        targetTurretAngle = Math.max(min, Math.min(max, angleDegrees));
        targetTicks =  targetTurretAngle * ticksPerDegree;
    }


    public double getTargetAngle() { return targetTurretAngle; }

    public double getCurrentAngle() {
        return robot.turretMotor.getCurrentPosition() / ticksPerDegree;
    }


    public boolean atTarget(double toleranceDegrees) {
        return Math.abs(getCurrentAngle() - targetTurretAngle) < toleranceDegrees;
    }
}