package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.customPathing.Drive;
import org.firstinspires.ftc.teamcode.customPathing.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
@TeleOp(name="Custom Pathing Tuning", group="Tuners")
public class CustomPathingTuning extends OpMode {
    RobotHardwareMap robot;
    Follower follower;
    Drive drive;






    public static double targetX = 72.0;
    public static double targetY = 96.0;
    public static double targetHeading = 0.0;
    public static boolean driveEnabled = false;






    private boolean started = false;






    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        robot.pinpointDriver.recalibrateIMU();

        drive = new Drive(robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(rConstants.FieldConstants.startingPose);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }






    @Override
    public void loop(){
        follower.update();

        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();
        double currentHeading = Math.toDegrees(follower.getPose().getHeading());

        if(driveEnabled){
            if(!started){
                drive.setTargetPoint(new Point(targetX, targetY, targetHeading));
                drive.trajectoryStartSequence();
                started = true;
            }

            drive.driveToTargetPoint(currentHeading, currentX, currentY);

            if(drive.isAtTarget(currentX, currentY)){
                drive.stopMotor();
                driveEnabled = false;
                started = false;
            }
        } else {
            drive.stopMotor();
            started = false;
        }

        telemetry.addData("Current X Position: ", "%.1f", currentX);
        telemetry.addData("Current Y Position: ", "%.1f", currentY);
        telemetry.addData("Current Heading: ", "%.1f", currentHeading);
        telemetry.addData("Driving: ", driveEnabled);
        telemetry.update();
    }
}