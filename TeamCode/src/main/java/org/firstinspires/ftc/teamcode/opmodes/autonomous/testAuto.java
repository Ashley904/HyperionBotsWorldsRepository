package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Test Auto", group="Autonomous")
public class testAuto extends LinearOpMode {
    Follower follower;
    FtcDashboard ftcDashboard;





    private Pose startingPose = new Pose(72.0, 72.0, Math.toRadians(0));
    private  Pose targetPose = new Pose(95.0, 72.0, Math.toRadians(0));





    private PathChain testPath;




    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        BuildPaths();

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(isStopRequested()) return;

            follower.update();
            CommandScheduler.getInstance().run();

            telemetry.addData("Current X Position: ", follower.getPose().getX());
            telemetry.addData("Current Y Position: ", follower.getPose().getY());
            telemetry.addData("Current Heading: ", follower.getPose().getHeading());
            telemetry.update();
        }
    }





    private void BuildPaths(){
        testPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startingPose.mirror().getX(), startingPose.mirror().getY()),
                        new Pose(targetPose.mirror().getX(), targetPose.mirror().getY())))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> follower.followPath(testPath))
                )
        );
    }
}
