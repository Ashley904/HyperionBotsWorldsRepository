package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@TeleOp(name="Spindexer Sensor Tuning", group="Tuning")
public class SpindexerSensorTuning extends OpMode {
    FtcDashboard ftcDashboard;





    private RobotHardwareMap robot;





    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardwareMap();
        robot.init(hardwareMap);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }





    @Override
    public void loop(){
        double leftDistanceSensorReading = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
        double rightDistanceSensorReading = robot.rightDistanceSensor.getDistance(DistanceUnit.CM);

        double spindexerLeftColorSensorReading = robot.leftSpindexerColorSensor.getDistance(DistanceUnit.CM);
        double spindexerRightColorSensorReading = robot.rightSpindexerColorSensor.getDistance(DistanceUnit.CM);



        telemetry.addData("Left Distance Sensor Reading: ", "%.1f", leftDistanceSensorReading);
        telemetry.addData("Right Distance Sensor Reading: ","%.1f", rightDistanceSensorReading);

        telemetry.addData("Left Spindexer Color Sensor Reading: ", "%.1f", spindexerLeftColorSensorReading);
        telemetry.addData("Right Spindexer Color Sensor Reading: ", "%.1f", spindexerRightColorSensorReading);
        telemetry.update();
    }
}
