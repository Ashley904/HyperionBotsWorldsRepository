package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@TeleOp(name="Odometry Tuning", group="Tuners")
public class OdometryTuning extends OpMode {
    FtcDashboard ftcDashboard;





    RobotHardwareMap robot;





    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
    }





    @Override
    public void init_loop(){
        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }





    @Override
    public void loop(){
        robot.pinpointDriver.update();

        double currentXPosition = robot.pinpointDriver.getPosX(DistanceUnit.INCH);
        double currentYPosition = robot.pinpointDriver.getPosY(DistanceUnit.INCH);
        double currentHeading = robot.pinpointDriver.getHeading(AngleUnit.DEGREES);

        double currentXVelocity = robot.pinpointDriver.getVelX();
        double currentYVelocity = robot.pinpointDriver.getVelY();






        telemetry.addData("Current X Position: ","%.0f", currentXPosition);
        telemetry.addData("Current Y Position: ","%.0f", currentYPosition);

        telemetry.addData("Current Heading: ","%.1f", currentHeading);

        telemetry.addData("Current X Velocity: ","%.1f", currentXVelocity);
        telemetry.addData("Current Y Velocity: ", "%.1f", currentYVelocity);
        telemetry.update();
    }
}
