package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.rConstants;

@TeleOp(name = "PedroPathing Testing Drive To Target")
public class pedroAutonomousTest extends OpMode {

    private Follower follower;

    // Target position (x, y, heading in RADIANS)
    private final Pose startPose = new Pose(72, 72, Math.toRadians(-180));
    private final Pose scorePose = new Pose(47.0, 85.0, Math.toRadians(135.0));

    private Path scorePreload;

    @Override
    public void init() {

        // Create follower
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose
        follower.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        telemetry.addLine("Initialized - ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        // This is what actually makes the robot move
        //follower.followPath(scorePreload);
    }

    @Override
    public void loop() {

        // REQUIRED: keeps path following running
        follower.update();

        // Telemetry for debugging
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading (rad)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}