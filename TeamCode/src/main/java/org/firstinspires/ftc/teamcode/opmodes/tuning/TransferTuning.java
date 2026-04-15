package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

@Config
@TeleOp(name="Transfer Tuning", group="Tuners")
public class TransferTuning extends OpMode {
    FtcDashboard ftcDashboard;





    private RobotHardwareMap robot;





    public static Boolean syncServos = true;





    public static double leftTransferServoPosition = 0.0;
    public static double rightTransferServoPosition = 0.0;
    public static double transferPosition = 0.0;





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
            robot.leftTransferServo.setPosition(transferPosition);
            robot.rightTransferServo.setPosition(transferPosition);
        }else if (!syncServos){
            robot.leftTransferServo.setPosition(leftTransferServoPosition);
            robot.rightTransferServo.setPosition(rightTransferServoPosition);
        }

        telemetry.addData("Sync servo status: ", syncServos);
        telemetry.update();
    }

}
