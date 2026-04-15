package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
@TeleOp(name="DriveTrain Tuning", group="Tuners")
public class DriveTrainTuning extends OpMode {
    FtcDashboard ftcDashboard;





    private RobotHardwareMap robot;





    public static boolean enableDrive = false;





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
    public void loop() {
        telemetry.clear();

        if (!enableDrive) {
            if (gamepad1.x) {
                robot.front_left_motor.setPower(1);
                telemetry.addLine("Front Left Motor");
            } else {
                robot.front_left_motor.setPower(0);
            }

            if (gamepad1.a) {
                robot.back_left_motor.setPower(1);
                telemetry.addLine("Back Left Motor");
            } else {
                robot.back_left_motor.setPower(0);
            }

            if (gamepad1.y) {
                robot.front_right_motor.setPower(1);
                telemetry.addLine("Front Right Motor");
            } else {
                robot.front_right_motor.setPower(0);
            }

            if (gamepad1.b) {
                robot.back_right_motor.setPower(1);
                telemetry.addLine("Back Right Motor");
            } else {
                robot.back_right_motor.setPower(0);
            }
        } else {
            RobotDrive();
        }

        telemetry.update();
    }





    private void RobotDrive(){
        double adjustedDrivingSpeed = rConstants.DriveTrainConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + rConstants.DriveTrainConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, rConstants.DriveTrainConstants.minimumDriveTrainSpeed);


        double y = gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x = -gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = gamepad1.right_stick_x * adjustedDrivingSpeed * -1;




        double frontLeftMotorPower = y + x + rx;
        double backLeftMotorPower = y - x + rx;
        double frontRightMotorPower = y - x - rx;
        double backRightMotorPower = y + x - rx;


        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftMotorPower), Math.abs(backLeftMotorPower)),
                Math.max(Math.abs(frontRightMotorPower), Math.abs(backRightMotorPower))
        );


        if(maxPower > rConstants.DriveTrainConstants.maximumDriveTrainSpeed){
            frontLeftMotorPower /= maxPower;
            backLeftMotorPower /= maxPower;
            frontRightMotorPower /= maxPower;
            backRightMotorPower /= maxPower;
        }


        robot.front_left_motor.setPower(frontLeftMotorPower);
        robot.back_left_motor.setPower(backLeftMotorPower);
        robot.front_right_motor.setPower(frontRightMotorPower);
        robot.back_right_motor.setPower(backRightMotorPower);
    }
}
