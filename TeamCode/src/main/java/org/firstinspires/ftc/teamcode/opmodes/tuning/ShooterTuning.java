package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.InitializeTransferCMD;
import org.firstinspires.ftc.teamcode.commands.ShootArtefactsCMD;
import org.firstinspires.ftc.teamcode.commands.TuningTransferCMD;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
@TeleOp(name="Shooter Calibration Point Tuning", group="Tuners")
public class ShooterTuning extends OpMode {
    FtcDashboard ftcDashboard;
    ShooterSubsystem shooterSubsystem;
    RobotHardwareMap robot;
    Follower follower;






    private ButtonReader selectAllianceButtonReader;
    private ButtonReader transferArtefactButtonReader;






    public static double targetFlyWheelVelocity = 0.0;
    private static Pose startingPose = new Pose(72.0, 72.0);
    public static double targetHoodPosition = rConstants.ShooterConstants.minimumHoodPosition;






    @Override
    public void init(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        robot.pinpointDriver.recalibrateIMU();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);


        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); }


        shooterSubsystem = new ShooterSubsystem(robot);
        rConstants.GamePadControls.gamepad1EX = new GamepadEx(gamepad1);
        selectAllianceButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.selectAlliance);
        transferArtefactButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.shootArtefacts);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();
    }






    @Override
    public void init_loop(){
        selectAllianceButtonReader.readValue();
        if(selectAllianceButtonReader.wasJustPressed()){
            if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.RED; }
            else if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.RED) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.BLUE; }
        }





        CommandScheduler.getInstance().schedule(new InitializeTransferCMD(robot));
        CommandScheduler.getInstance().run();




        telemetry.addData("Selected Alliance: ", rConstants.Enums.selectedAlliance);
        telemetry.addData("Cycle Alliance: ", "A");
        telemetry.update();
    }





    @Override
    public void start() { CommandScheduler.getInstance().reset(); }






    @Override
    public void loop(){
        follower.update();



        shooterSubsystem.setTargetVelocity(targetFlyWheelVelocity);
        shooterSubsystem.setHoodPosition(targetHoodPosition);
        shooterSubsystem.periodic();



        //----------Shoot Artefact----------//
        transferArtefactButtonReader.readValue();
        if(transferArtefactButtonReader.wasJustPressed()) { CommandScheduler.getInstance().schedule(new TuningTransferCMD(robot)); }
        CommandScheduler.getInstance().run();
        //----------end----------//



        telemetry.addData("Selected Alliance: ", rConstants.Enums.selectedAlliance);
        telemetry.addData("X Pose: ", "%.1f", getXPose());
        telemetry.addData("Y Pose: ", "%.1f", getYPose());
        telemetry.addData("Distance To Goal: ", "%.1f", getDistanceToGoal());
        telemetry.addData("Current FlyWheel Velocity: ", "%.0f", shooterSubsystem.getCurrentFlyWheelVelocity());
        telemetry.addData("Target FlyWheel Velocity: ", "%.0f", targetFlyWheelVelocity);
        telemetry.update();
    }






    private Pose getActiveGoalPose() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalPose
                : rConstants.FieldConstants.redGoalPose;
    }
    private double getXPose() { return follower.getPose().getX(); }
    private double getYPose() { return follower.getPose().getY(); }
    private double getDistanceToGoal() {
        return follower.getPose().distanceFrom(getActiveGoalPose());
    }
}