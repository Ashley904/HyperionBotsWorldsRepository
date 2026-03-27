package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShootArtefactsCMD;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

import org.firstinspires.ftc.teamcode.util.rConstants;

@TeleOp(name="Worlds Rebuild TeleOp", group="TeleOp")
public class WorldsRebuildTeleOp extends OpMode {
    //PDController headingPDController;





    IntakeSubsystem intakeSubsystem;
    SpindexerSubsystem spindexerSubsystem;





    ShootArtefactsCMD shootArtefactsCMD;





    private RobotHardwareMap robot;
    //private Follower follower;





    private ButtonReader cycleDriveModesButtonReader;
    private ButtonReader enableIntakeButtonReader, reverseIntakeButtonReader;
    private ButtonReader shootArtefactsButtonReader;





    private double targetHeading=0.0;
    private boolean headingCorrectionEnabled=true;





    @Override
    public void init(){
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);






        intakeSubsystem = new IntakeSubsystem(robot);
        spindexerSubsystem = new SpindexerSubsystem(robot);
        //headingPDController = new PDController(rConstants.DriveTrainConstants.headingKp, rConstants.DriveTrainConstants.headingKd);





        shootArtefactsCMD = new ShootArtefactsCMD(spindexerSubsystem);





        //Calling Functions
        InitializeGamePadControls();





        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled);
        spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.Intaking);





        telemetry.addData("Status: ", "Initialization Complete...");
        telemetry.update();
    }





    @Override
    public void init_loop(){
        intakeSubsystem.periodic();
        spindexerSubsystem.periodic();

        telemetry.addData("Status: ", "Waiting for start...");
        telemetry.update();
    }





    @Override
    public void loop(){
        //Calling Functions
        TelemetryUpdating();
        GamepadControlsManaging();
        BackgroundOperations();
        RobotDrive();
    }





    private void RobotDrive(){
        double adjustedDrivingSpeed = rConstants.DriveTrainConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + rConstants.DriveTrainConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, rConstants.DriveTrainConstants.minimumDriveTrainSpeed);


        double y = -rConstants.GamePadControls.gamepad1EX.getLeftY() * adjustedDrivingSpeed;
        double x = rConstants.GamePadControls.gamepad1EX.getLeftX() * adjustedDrivingSpeed;
        double rx = rConstants.GamePadControls.gamepad1EX.getRightX() * adjustedDrivingSpeed * -1;
        rx = rx * 1.1;


        /*
        double rotatedX = x * Math.cos(-getHeading()) - y * Math.sin(-getHeading());
        double rotatedY = x * Math.sin(-getHeading()) + y * Math.cos(-getHeading());
        rotatedX = rotatedX * 1.1;




        //----------Heading Correction----------//
        if(Math.abs(rConstants.GamePadControls.gamepad1EX.getRightX()) > 0.05) {
            headingCorrectionEnabled = false;
            targetHeading = getHeading();
        } else { headingCorrectionEnabled = true; }

        //if(headingCorrectionEnabled) { rx = headingPDController.calculate(getHeading(), targetHeading, -0.75, 0.75); }
        //----------end-----------//

         */






        double frontLeftMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y + x + rx) : (y + x + rx);
        double backLeftMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y - x + rx) : (y - x + rx);
        double frontRightMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y - x - rx) : (y - x - rx);
        double backRightMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y + x - rx) : (y + x - rx);


        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftMotorPower), Math.abs(backLeftMotorPower)),
                Math.max(Math.abs(frontRightMotorPower), Math.abs(frontRightMotorPower))
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
        //----------end-----------//





        //-----------Cycling Drive Modes-----------
        cycleDriveModesButtonReader.readValue();
        if(cycleDriveModesButtonReader.wasJustPressed() && rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric) { rConstants.Enums.selectedDriveMode = rConstants.Enums.DriveMode.FieldCentric; RumbleGamePad(200); }
        else if(cycleDriveModesButtonReader.wasJustPressed() && rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.FieldCentric) { rConstants.Enums.selectedDriveMode = rConstants.Enums.DriveMode.RobotCentric; RumbleGamePad(200); }
        //----------end----------//





        //----------Cycling Artefacts----------//
        shootArtefactsButtonReader.readValue();
        if(shootArtefactsButtonReader.wasJustPressed()) {
            shootArtefactsCMD = new ShootArtefactsCMD(spindexerSubsystem);
            CommandScheduler.getInstance().schedule(shootArtefactsCMD);

            RumbleGamePad(200);
        }
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
        intakeSubsystem.periodic();
        spindexerSubsystem.periodic();

        CommandScheduler.getInstance().run();
    }






    private void TelemetryUpdating(){
        telemetry.addData("Selected Drive Mode: ", rConstants.Enums.selectedDriveMode);

        /*
        telemetry.addData("X Pose: ",  "%.1f" , getXPose());
        telemetry.addData("Y Pose: ",  "%.1f" , getYPose());
        telemetry.addData("X Velocity: ", "%.2f" , getXVelocity());
        telemetry.addData("Y Velocity: ", "%.2f" , getYVelocity());
        telemetry.addData("Current Heading: ", "%.1f" , Math.toDegrees(getHeading()));

         */

        telemetry.addData("Intake State: ", intakeSubsystem.getState());
        telemetry.addData("Intake Velocity: ", "%.0f" , intakeSubsystem.getIntakeVelocity());
        telemetry.addData("Intake Current Draw: ", "%.1f",  intakeSubsystem.getIntakeCurrentDraw());


        telemetry.addData("Current Spindexer State: ", spindexerSubsystem.getSpindexerState());
        telemetry.addData("Spindexer Position Reached: ", spindexerSubsystem.spindexerPositionReached());
        telemetry.addData("Spindexer Velocity: ", "%.1f", spindexerSubsystem.getVelocity());

        telemetry.update();
    }





    private void RumbleGamePad(int duration){
        gamepad1.rumble(1.0, 1.0, duration);
    }





    //Helpers
    /*
    private double getHeading() { return follower.getPose().getHeading(); }
    private double getXPose() { return follower.getPose().getX(); }
    private double getYPose() { return follower.getPose().getY(); }

    private double getXVelocity() { return follower.getVelocity().getXComponent(); }
    private double getYVelocity() { return follower.getVelocity().getYComponent(); }

     */
}
