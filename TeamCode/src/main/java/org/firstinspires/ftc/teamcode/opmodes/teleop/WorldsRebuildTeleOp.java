package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.InitializeSpindexerCMD;
import org.firstinspires.ftc.teamcode.commands.InitializeTransferCMD;
import org.firstinspires.ftc.teamcode.commands.ShootArtefactsCMD;
import org.firstinspires.ftc.teamcode.controllers.PDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
@TeleOp(name="Worlds Rebuild TeleOp", group="TeleOp")
public class WorldsRebuildTeleOp extends OpMode {
    PDController headingPDController;





    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    SpindexerSubsystem spindexerSubsystem;





    ShootArtefactsCMD shootArtefactsCMD;





    private RobotHardwareMap robot;
    private Follower follower;





    private ButtonReader cycleDriveModesButtonReader;
    private ButtonReader enableIntakeButtonReader, reverseIntakeButtonReader;
    private ButtonReader shootArtefactsButtonReader;





    private double targetHeading=0.0, headingOffset=0.0;
    private boolean headingCorrectionEnabled=true;





    public static double targetFlyWheelVelocity = 5;
    public static double targetHoodPosition = 0.0;





    public static double spindexerPosition = 0.0;





    private int loopCount = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();





    @Override
    public void init(){
        // Enabling Bulk Reading
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }





        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72.0, 72.0));






        robot = new RobotHardwareMap();
        robot.init(hardwareMap);






        intakeSubsystem = new IntakeSubsystem(robot);
        shooterSubsystem = new ShooterSubsystem(robot);
        spindexerSubsystem = new SpindexerSubsystem(robot);
        headingPDController = new PDController(rConstants.DriveTrainConstants.headingKp, rConstants.DriveTrainConstants.headingKd);





        //Calling Functions
        InitializeGamePadControls();




        rConstants.Enums.selectedDriveMode = rConstants.Enums.DriveMode.FieldCentric;
        rConstants.Enums.currentShooterState = rConstants.Enums.ShooterState.Disabled;
        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled);




        // Initializing SubsystemsshooterSubsystem.setHoodPosition(targetHoodPosition);
        shooterSubsystem.setTargetVelocity(  0);

        CommandScheduler.getInstance().schedule(new InitializeTransferCMD(robot));
        CommandScheduler.getInstance().schedule(new InitializeSpindexerCMD(robot, spindexerSubsystem));






        telemetry.addData("Status: ", "Initialization Complete...");
        telemetry.update();
    }





    @Override
    public void init_loop(){
        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();

        telemetry.addData("Status: ", "Waiting for start...");
        telemetry.update();
    }





    @Override
    public void loop(){
        //Calling Functions
        GamepadControlsManaging();
        RobotDrive();
        BackgroundOperations();

        if(loopCount++ % 3 == 0) TelemetryUpdating();
    }





    private void RobotDrive(){
        double adjustedDrivingSpeed = rConstants.DriveTrainConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + rConstants.DriveTrainConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, rConstants.DriveTrainConstants.minimumDriveTrainSpeed);


        double y = -rConstants.GamePadControls.gamepad1EX.getLeftY() * adjustedDrivingSpeed;
        double x = rConstants.GamePadControls.gamepad1EX.getLeftX() * adjustedDrivingSpeed;
        double rx = rConstants.GamePadControls.gamepad1EX.getRightX() * adjustedDrivingSpeed * -1;
        rx = rx * 1.1;


        double rotatedX = x * Math.cos(-getFieldHeading()) - y * Math.sin(-getFieldHeading());
        double rotatedY = x * Math.sin(-getFieldHeading()) + y * Math.cos(-getFieldHeading());
        rotatedX = rotatedX * 1.1;




        //----------Heading Correction----------//
        if(Math.abs(rConstants.GamePadControls.gamepad1EX.getRightX()) > 0.05) {
            headingCorrectionEnabled = false;
            targetHeading = getHeading();
        } else { headingCorrectionEnabled = true; }

        if(headingCorrectionEnabled) { rx = headingPDController.calculate(getHeading(), targetHeading, -0.75, 0.75); }
        //----------end-----------//






        double frontLeftMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y + x + rx) : (rotatedY + rotatedX + rx);
        double backLeftMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y - x + rx) : (rotatedY - rotatedX + rx);
        double frontRightMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y - x - rx) : (rotatedY - rotatedX - rx);
        double backRightMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y + x - rx) : (rotatedY + rotatedX - rx);


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





    private void GamepadControlsManaging(){
        //----------Intaking Managing----------//
        if(spindexerSubsystem.getSpindexerState() != SpindexerSubsystem.SpindexerState.Shooting){
            enableIntakeButtonReader.readValue();
            reverseIntakeButtonReader.readValue();

            if(enableIntakeButtonReader.wasJustPressed()
                    && intakeSubsystem.getState() != IntakeSubsystem.IntakeState.IntakingUnJamming
                    && intakeSubsystem.getState() != IntakeSubsystem.IntakeState.OuttakingUnJamming){ //Intaking Control
                switch(intakeSubsystem.getState()){
                    case Disabled:
                    case Reversing:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking);
                        break;

                    case Intaking:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled);
                        break;
                }

                RumbleGamePad(150);
            }


            if(reverseIntakeButtonReader.wasJustPressed()
                    && intakeSubsystem.getState() != IntakeSubsystem.IntakeState.IntakingUnJamming
                    && intakeSubsystem.getState() != IntakeSubsystem.IntakeState.OuttakingUnJamming){ // Reversing Control
                switch(intakeSubsystem.getState()){
                    case Disabled:
                    case Intaking:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Reversing);
                        break;

                    case Reversing:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled);
                        break;
                }

                RumbleGamePad(150);
            }
        }
        //----------end-----------//





        //-----------Cycling Drive Modes-----------//
        cycleDriveModesButtonReader.readValue();
        if(cycleDriveModesButtonReader.wasJustPressed() && rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric) { rConstants.Enums.selectedDriveMode = rConstants.Enums.DriveMode.FieldCentric; RumbleGamePad(200); }
        else if(cycleDriveModesButtonReader.wasJustPressed() && rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.FieldCentric) { rConstants.Enums.selectedDriveMode = rConstants.Enums.DriveMode.RobotCentric; RumbleGamePad(200); }
        //----------end----------//





        //----------Cycle Artefacts----------//
        shootArtefactsButtonReader.readValue();
        if(shootArtefactsButtonReader.wasJustPressed()) { CommandScheduler.getInstance().schedule(new ShootArtefactsCMD(spindexerSubsystem, intakeSubsystem, robot)); }
        //----------end----------//
    }
    private void InitializeGamePadControls(){
        rConstants.GamePadControls.gamepad1EX = new GamepadEx(gamepad1);
        rConstants.GamePadControls.gamepad2EX = new GamepadEx(gamepad2);

        cycleDriveModesButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.cycleDriveModes);
        enableIntakeButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.enableIntake);
        reverseIntakeButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.reverseIntake);
        shootArtefactsButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.shootArtefacts);
    }





    private void BackgroundOperations(){
        follower.update();
        CommandScheduler.getInstance().run();



        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();



        shooterSubsystem.setTargetVelocity(targetFlyWheelVelocity);
        shooterSubsystem.setHoodPosition(targetHoodPosition);
    }






    private void TelemetryUpdating(){
        telemetry.addData("Selected Drive Mode: ", rConstants.Enums.selectedDriveMode);

        telemetry.addData("X Pose: ",  "%.1f" , getXPose());
        telemetry.addData("Y Pose: ",  "%.1f" , getYPose());
        telemetry.addData("X Velocity: ", "%.2f" , getXVelocity());
        telemetry.addData("Y Velocity: ", "%.2f" , getYVelocity());
        telemetry.addData("Current Heading: ", "%.1f" , Math.toDegrees(getHeading()));

        telemetry.addData("Intake State: ", intakeSubsystem.getState());
        telemetry.addData("Intake Velocity: ", "%.0f" , intakeSubsystem.getIntakeVelocity());
        telemetry.addData("Intake Current Draw: ", "%.1f",  intakeSubsystem.getIntakeCurrentDraw());

        telemetry.addData("Spindexer State: ", spindexerSubsystem.getSpindexerState());
        telemetry.addData("Spindexer Velocity: ", spindexerSubsystem.getVelocity());
        telemetry.addData("Spindexer Position: ", spindexerSubsystem.getPosition());
        telemetry.addData("Spindexer Position Reached?: ", spindexerSubsystem.spindexerPositionReached());

        telemetry.addData("Current FlyWheel State: ", rConstants.Enums.currentShooterState);
        telemetry.addData("Current FlyWheel Velocity: ", "%.0f", shooterSubsystem.getCurrentFlyWheelVelocity());
        telemetry.addData("Target FlyWheel Velocity", "%.0f", targetFlyWheelVelocity);

        telemetry.addData("Loop Time: ", elapsedTime.milliseconds());
        elapsedTime.reset();

        telemetry.update();
    }





    private void RumbleGamePad(int duration){
        gamepad1.rumble(1.0, 1.0, duration);
    }





    //Helpers
    private double getHeading() { return follower.getPose().getHeading(); }
    private double getXPose() { return follower.getPose().getX(); }
    private double getYPose() { return follower.getPose().getY(); }
    private double getFieldHeading() {
        if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE) headingOffset = rConstants.AllianceHeadingOffsets.blueAllianceHeadingOffset;
        else if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.RED) headingOffset = rConstants.AllianceHeadingOffsets.redAllianceHeadingOffset;

        return getHeading() - headingOffset;
    }

    private double getXVelocity() { return follower.getVelocity().getXComponent(); }
    private double getYVelocity() { return follower.getVelocity().getYComponent(); }
}