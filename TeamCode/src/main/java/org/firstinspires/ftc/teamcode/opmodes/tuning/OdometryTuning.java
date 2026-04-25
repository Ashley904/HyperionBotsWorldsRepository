package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

@TeleOp(name="Odometry Tuning", group="Tuners")
public class OdometryTuning extends OpMode {
    FtcDashboard ftcDashboard;





    RobotHardwareMap robot;
    Follower follower;





    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardwareMap();
        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(rConstants.FieldConstants.startingPose);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }





    @Override
    public void loop(){
        follower.update();






        double currentHeading = follower.getPose().getHeading();
        double currentXVelocity = follower.getVelocity().getXComponent();
        double currentYVelocity = follower.getVelocity().getYComponent();






        telemetry.addData("Selected Alliance: ", rConstants.Enums.selectedAlliance);
        telemetry.addData("Current X Position: ","%.0f", getXPose());
        telemetry.addData("Current Y Position: ","%.0f", getYPose());
        telemetry.addData("Current Heading: ","%.1f", Math.toDegrees(currentHeading));
        telemetry.addData("Current X Velocity: ","%.1f", currentXVelocity);
        telemetry.addData("Current Y Velocity: ", "%.1f", currentYVelocity);
        telemetry.addData("Distance To Goal: ", "%.1f", getDistanceToGoal());
        telemetry.update();
    }





    private double getDistanceToGoal() {
        Pose activeGoal = getActiveGoalPose();

        double dx = activeGoal.getX() - getXPose();
        double dy = activeGoal.getY() - getYPose();

        return Math.sqrt(dx * dx + dy * dy);
    }





    // FIX: Imported from your main TeleOp to keep distance calculations consistent
    private Pose getActiveGoalPose() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalPose
                : rConstants.FieldConstants.redGoalPose;
    }





    // FIX: X and Y getters were previously swapped.
    private double getXPose() { return follower.getPose().getX(); }
    private double getYPose() { return follower.getPose().getY(); }
}