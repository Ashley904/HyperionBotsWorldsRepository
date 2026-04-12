package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@Config
@TeleOp(name="Intake Tuning", group="Tuners")
public class IntakeTuning extends OpMode {
    FtcDashboard ftcDashboard;





    private RobotHardwareMap robot;





    public static double intakeSpeed;





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
        robot.intakeMotor.setPower(intakeSpeed);

        double intakeVelocity = robot.intakeMotor.getVelocity();
        double intakeCurrentDraw = robot.intakeMotor.getCurrent(CurrentUnit.AMPS);

        telemetry.addData("Intake Velocity: ", intakeVelocity);
        telemetry.addData("Intake Current Draw: ", intakeCurrentDraw);
        telemetry.update();
    }
}
