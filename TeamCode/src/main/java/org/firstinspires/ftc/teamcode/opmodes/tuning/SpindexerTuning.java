package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@Config
@TeleOp(name="Spindexer Tuning", group="Tuners")
public class SpindexerTuning extends OpMode {
    FtcDashboard ftcDashboard;





    private RobotHardwareMap robot;





    public static Boolean syncServos = true;





    public static double leftSpindexerServoPosition = 0.0;
    public static double rightSpindexerServoPosition = 0.0;
    public static double spindexerPosition = 0.0;





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
        if(syncServos) {
            robot.leftSpindexerServo.setPosition(spindexerPosition);
            robot.rightSpindexerServo.setPosition(spindexerPosition);
        }else {
            robot.leftSpindexerServo.setPosition(leftSpindexerServoPosition);
            robot.rightSpindexerServo.setPosition(rightSpindexerServoPosition);
        }

        telemetry.addData("Current Spindexer Encoder Position: ", robot.spindexerEncoder.getCurrentPosition());
        telemetry.addData("Sync servo status: ", syncServos);
        telemetry.update();
    }

}
