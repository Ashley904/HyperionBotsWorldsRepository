package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose; // FIX: Imported for active goal tracking
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
@TeleOp(name="Shooter Calibration Point Tuning", group="Tuners")
public class ShooterTuning extends OpMode {
    FtcDashboard ftcDashboard;
    ShooterSubsystem shooterSubsystem;
    RobotHardwareMap robot;
    Follower follower;





    public static double targetFlyWheelVelocity = 0.0;
    public static double targetHoodPosition = rConstants.ShooterConstants.minimumHoodPosition;



    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(rConstants.FieldConstants.startingPose);





        robot = new RobotHardwareMap();
        robot.init(hardwareMap);





        shooterSubsystem = new ShooterSubsystem(robot);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }






    @Override
    public void loop(){
        follower.update();
        robot.pinpointDriver.update();

        shooterSubsystem.setTargetVelocity(targetFlyWheelVelocity);
        shooterSubsystem.setHoodPosition(targetHoodPosition);

        shooterSubsystem.periodic();





        telemetry.addData("Selected Alliance: ", rConstants.Enums.selectedAlliance);
        telemetry.addData("Distance To Goal: ", "%.1f", getDistanceToGoal()); // FIX: Label updated
        telemetry.addData("Current FlyWheel Velocity: ", "%.0f", shooterSubsystem.getCurrentFlyWheelVelocity());
        telemetry.update();
    }






    private Pose getActiveGoalPose() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalPose
                : rConstants.FieldConstants.redGoalPose;
    }






    private double getXPose() { return follower.getPose().getX(); }
    private double getYPose() { return follower.getPose().getY(); }





    private double getDistanceToGoal() {
        double goalX = getGoalX();
        double goalY = getGoalY();

        double dx = goalX - getXPose();
        double dy = goalY - getYPose();

        return Math.sqrt(dx * dx + dy * dy);
    }




    private double getGoalX() {
        return getActiveGoalPose().getX();
    }
    private double getGoalY() {
        return getActiveGoalPose().getY();
    }
}