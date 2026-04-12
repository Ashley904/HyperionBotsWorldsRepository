package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@TeleOp(name="DriveTrain Tuning", group="Tuners")
public class DriveTrainTuning extends OpMode {
    FtcDashboard ftcDashboard;





    private RobotHardwareMap robot;





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
        if(gamepad1.x) { //FL
            robot.front_left_motor.setPower(1);
            telemetry.addLine("Front Left Motor");
        }
        else robot.front_left_motor.setPower(0);



        if(gamepad1.a) { //BL
            robot.back_left_motor.setPower(1);
            telemetry.addLine("Back Left Motor");
        }
        else robot.back_left_motor.setPower(0);



        if(gamepad1.y) { //FR
            robot.front_left_motor.setPower(1);
            telemetry.addLine("Front Right Motor");
        }
        else robot.front_right_motor.setPower(0);



        if(gamepad1.b) { //BR
            robot.back_left_motor.setPower(1);
            telemetry.addLine("Back Right Motor");
        }
        else robot.back_right_motor.setPower(0);





        telemetry.update();
    }
}
