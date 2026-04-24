package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.PDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
@TeleOp(name="DriveTrain Heading Tuner", group="Tuners")
public class DriveHeadingPIDTuning extends OpMode {
    private RobotHardwareMap robot;
    private Follower follower;
    private PDController pdController;





    public static double targetHeading = 0.0, currentHeading = 0.0;





    @Override
    public void init(){
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        pdController = new PDController(rConstants.DriveTrainConstants.headingKp, rConstants.DriveTrainConstants.headingKd);
    }





    @Override
    public void init_loop(){
        telemetry.addData("Alliance: ", rConstants.Enums.selectedAlliance);
        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }




    @Override
    public void loop(){
        follower.update();
        currentHeading = Math.toDegrees(getHeading());

        // Calling Functions
        TelemetryUpdating();
        RobotHeadingCorrection();

        pdController.setGains(rConstants.DriveTrainConstants.headingKp, rConstants.DriveTrainConstants.headingKd);
    }





    private void RobotHeadingCorrection(){
        double error = Math.toRadians(targetHeading) - getHeading();
        while (error >  Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        // PD output becomes pure rotation — no x/y translation
        double rx = pdController.calculate(0, error, -0.85, 0.85);
        RotateInPlace(rx);
    }
    private void RotateInPlace(double rx){
        robot.front_left_motor.setPower(rx);
        robot.back_left_motor.setPower(rx);
        robot.front_right_motor.setPower(-rx);
        robot.back_right_motor.setPower(-rx);
    }





    private double getAngleToGoal() {
        double deltaX = getActiveGoalPose().getX() - getCurrentX();
        double deltaY = getActiveGoalPose().getY() - getCurrentY();
        return Math.atan2(deltaY, deltaX);
    }
    public boolean isAimedAtGoal() {
        double error = getAngleToGoal() - getHeading();
        while(error > Math.PI) error -= 2 * Math.PI;
        while(error < -Math.PI) error += 2 * Math.PI;
        return Math.abs(error) <= rConstants.DriveTrainConstants.autoLockHeadingTolerance;
    }






    private void TelemetryUpdating(){
        telemetry.addData("Target Heading: ","%.1f" , targetHeading);
        telemetry.addData("Current Heading: ","%.1f" , currentHeading);

        telemetry.addData("X: ", "%.1f", getCurrentX());
        telemetry.addData("Y: ", "%.1f", getCurrentY());

        telemetry.addData("Angle to goal: ", "%.1f", Math.toDegrees(getAngleToGoal()));
        telemetry.addData("Is aimed at goal?: ", isAimedAtGoal());
        telemetry.update();
    }






    private double getCurrentX() { return follower.getPose().getX(); }
    private double getCurrentY() { return follower.getPose().getY();}
    private double getHeading() { return follower.getPose().getHeading(); }





    private Pose getActiveGoalPose() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalPose
                : rConstants.FieldConstants.redGoalPose;
    }
}
