package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.controllers.PDController;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class TurretSubsystem extends SubsystemBase {
    private final RobotHardwareMap robot;
    private final PDController pdController;





    private int targetTurretAngle;





    private static final double ticksPerRevolution = 537.7;
    private static final double gearRatio = 172.0 / 35.0;
    private static final double ticksPerRadian = (ticksPerRevolution / (2 * Math.PI)) * gearRatio;




    public TurretSubsystem(RobotHardwareMap robot){
        this.robot = robot;
        pdController = new PDController(
                rConstants.TurretConstants.turretKp,
                rConstants.TurretConstants.turretKd
        );
    }





    @Override
    public void periodic(){
        double adjustment = pdController.calculate(getCurrentTurretPosition(), getTargetTurretPosition(), -rConstants.TurretConstants.maxTurretPower, rConstants.TurretConstants.maxTurretPower);
        double clampedAdjustment = Math.max(
                -rConstants.TurretConstants.maxTurretPower,
                Math.min(rConstants.TurretConstants.maxTurretPower, adjustment)
        );

        robot.turretMotor.setPower(clampedAdjustment);
    }





    // Helper Functions
    public void setTurretAngle(int targetTurretAngle) { this.targetTurretAngle = targetTurretAngle; }

    public double getCurrentTurretPosition() { return robot.turretMotor.getCurrentPosition(); }
    public double getTargetTurretPosition() { return Math.toRadians(targetTurretAngle) * ticksPerRadian;}
}
