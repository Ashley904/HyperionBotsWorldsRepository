package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
@TeleOp(name="Shooter Calibration Point Tuning", group="Tuners")
public class ShooterTuning extends OpMode {
    FtcDashboard ftcDashboard;
    ShooterSubsystem shooterSubsystem;
    RobotHardwareMap robot;





    public static double targetFlyWheelVelocity = 0.0;
    public static double targetHoodPosition = rConstants.ShooterConstants.minimumHoodPosition;



    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());






        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        robot.pinpointDriver.recalibrateIMU();
        robot.pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));





        shooterSubsystem = new ShooterSubsystem(robot);
    }

    @Override
    public void loop(){
        robot.pinpointDriver.update();

        shooterSubsystem.setTargetVelocity(targetFlyWheelVelocity);
        shooterSubsystem.setHoodPosition(targetHoodPosition);

        shooterSubsystem.periodic();





        double fieldX = 72.0 + robot.pinpointDriver.getPosX(DistanceUnit.INCH);
        double fieldY = 72.0 + robot.pinpointDriver.getPosY(DistanceUnit.INCH);

        double dx = rConstants.FieldConstants.blueGoalXPosition - fieldX;
        double dy = rConstants.FieldConstants.blueGoalYPosition - fieldY;
        double distanceToGoal = Math.sqrt(dx * dx + dy * dy);






        telemetry.addData("Distance To Blue Goal: ", "%.1f", distanceToGoal);
        telemetry.addData("Current FlyWheel Velocity: ", "%.0f", shooterSubsystem.getCurrentFlyWheelVelocity());
        telemetry.update();
    }
}