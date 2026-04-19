package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@TeleOp(name="Odometry Tuning", group="Tuners")
public class OdometryTuning extends OpMode {
    FtcDashboard ftcDashboard;





    RobotHardwareMap robot;





    private final Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 72.0, 72.0, AngleUnit.DEGREES, 0);





    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardwareMap();
        robot.init(hardwareMap);

        robot.pinpointDriver.resetPosAndIMU();
        robot.pinpointDriver.recalibrateIMU();
        robot.pinpointDriver.setPosition(startingPose);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }





    @Override
    public void loop(){
        robot.pinpointDriver.update();

        double currentXPosition = 72.0 + robot.pinpointDriver.getPosY(DistanceUnit.INCH);
        double currentYPosition = 72.0 + robot.pinpointDriver.getPosX(DistanceUnit.INCH);
        double currentHeading = robot.pinpointDriver.getHeading(AngleUnit.RADIANS);

        double currentXVelocity = robot.pinpointDriver.getVelY();
        double currentYVelocity = robot.pinpointDriver.getVelX();





        telemetry.addData("Current X Position: ","%.0f", currentXPosition);
        telemetry.addData("Current Y Position: ","%.0f", currentYPosition);

        telemetry.addData("Current Heading: ","%.1f", Math.toDegrees(currentHeading));

        telemetry.addData("Current X Velocity: ","%.1f", currentXVelocity);
        telemetry.addData("Current Y Velocity: ", "%.1f", currentYVelocity);
        telemetry.update();
    }
}
