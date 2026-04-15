package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@Config
@TeleOp(name="Turret Tuning", group="Tuners")
public class TurretTuning extends OpMode {
    FtcDashboard ftcDashboard;
    RobotHardwareMap robot;
    TurretSubsystem turretSubsystem;





    public static int targetTurretAngle = 0;





    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardwareMap();
        robot.init(hardwareMap);

        turretSubsystem = new TurretSubsystem(robot);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }





    @Override
    public void loop(){
        turretSubsystem.setTurretAngle(targetTurretAngle);
        turretSubsystem.periodic();


        telemetry.addData("Current Turret Position: ", turretSubsystem.getCurrentTurretPosition());
        telemetry.addData("Target Turret Position: ", turretSubsystem.getTargetTurretPosition());
        telemetry.update();
    }
}
