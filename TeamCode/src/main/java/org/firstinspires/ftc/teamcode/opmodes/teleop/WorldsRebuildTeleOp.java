package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.InitializeSpindexerCMD;
import org.firstinspires.ftc.teamcode.commands.InitializeTransferCMD;
import org.firstinspires.ftc.teamcode.commands.ShootArtefactsCMD;
import org.firstinspires.ftc.teamcode.controllers.PDController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;

import org.firstinspires.ftc.teamcode.util.rConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;

@Config
@TeleOp(name="Worlds Rebuild TeleOp", group="TeleOp")
public class WorldsRebuildTeleOp extends OpMode {
    FtcDashboard ftcDashboard;
    PDController headingPDController;





    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    SpindexerSubsystem spindexerSubsystem;
    SpindexerSensorsSubsystem spindexerSensorsSubsystem;





    ShootArtefactsCMD shootArtefactsCMD;





    private RobotHardwareMap robot;





    private ButtonReader cycleDriveModesButtonReader;
    private ButtonReader enableIntakeButtonReader, reverseIntakeButtonReader;
    private ButtonReader shootArtefactsButtonReader;
    private  ButtonReader indexSpindexerButtonReader;





    private double targetHeading=0.0, headingOffset=0.0;
    private boolean headingCorrectionEnabled=true;





    public static double spindexerPosition = 0.0;





    private int loopCount = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();





    private static final List<CalibrationPoints> calibrationPoints = new ArrayList<>();
    double dynamicTargetFlyWheelVelocity = 0.0, dynamicTargetHoodPosition = 0.0;





    @Override
    public void init(){
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        robot.pinpointDriver.recalibrateIMU();
        robot.pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));




        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());





        // Enabling Bulk Reading
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }





        intakeSubsystem = new IntakeSubsystem(robot);
        shooterSubsystem = new ShooterSubsystem(robot);
        spindexerSubsystem = new SpindexerSubsystem(robot);
        spindexerSensorsSubsystem = new SpindexerSensorsSubsystem(robot);
        headingPDController = new PDController(rConstants.DriveTrainConstants.headingKp, rConstants.DriveTrainConstants.headingKd);





        //Calling Functions
        InitializeGamePadControls();
        InitializeCalibrationPoints();




        rConstants.Enums.selectedDriveMode = rConstants.Enums.DriveMode.FieldCentric;
        rConstants.Enums.currentShooterState = rConstants.Enums.ShooterState.Disabled;
        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled);




        shooterSubsystem.setHoodPosition(rConstants.ShooterConstants.minimumHoodPosition);
        shooterSubsystem.setTargetVelocity(0);

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
    public void start() { CommandScheduler.getInstance().reset();  }





    @Override
    public void loop(){
        //Calling Functions
        GamepadControlsManaging();
        BackgroundOperations();
        RobotDrive();
        CalculateShooterParameters();

        if(loopCount++ % 3 == 0) TelemetryUpdating();
    }





    private void RobotDrive(){
        double adjustedDrivingSpeed = rConstants.DriveTrainConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + rConstants.DriveTrainConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, rConstants.DriveTrainConstants.minimumDriveTrainSpeed);


        double y = gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x = gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = gamepad1.right_stick_x * adjustedDrivingSpeed * -1;
        x = x * 1.1;


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
        indexSpindexerButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.indexSpindexer);
    }





    private void BackgroundOperations(){
        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();

        shooterSubsystem.setTargetVelocity(dynamicTargetFlyWheelVelocity);
        shooterSubsystem.setHoodPosition(dynamicTargetHoodPosition);

        SpindexerManaging();

        robot.pinpointDriver.update();
        CommandScheduler.getInstance().run();
    }
    private void SpindexerManaging(){
        if(spindexerSubsystem.getSpindexerState() == SpindexerSubsystem.SpindexerState.Shooting
        || intakeSubsystem.getState() != IntakeSubsystem.IntakeState.Intaking
        || !spindexerSubsystem.spindexerPositionReached()) return;


        spindexerSensorsSubsystem.readSensors();
        if(spindexerSensorsSubsystem.spindexerIsFull()) { intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled); return; }


        int currentFrontSlot = spindexerSensorsSubsystem.getCurrentFrontSlot();
        if(spindexerSensorsSubsystem.isSlotOccupied(currentFrontSlot)){
            int nextEmpty = spindexerSensorsSubsystem.getFirstEmptySlot();
            if(nextEmpty != -1){
                spindexerSubsystem.setSpindexerPosition(
                        rConstants.SpindexerConstants.intakingPositions[nextEmpty]
                );
            }
        }
    }





    private static class CalibrationPoints {
        public double distanceToGoal;
        public double flyWheelVelocity;
        public double hoodPosition;

        public CalibrationPoints(double distanceToGoal, double flyWheelVelocity, double hoodPosition) {
            this.distanceToGoal   = distanceToGoal;
            this.flyWheelVelocity = flyWheelVelocity;
            this.hoodPosition     = hoodPosition;
        }
    }
    private void InitializeCalibrationPoints() {
        calibrationPoints.clear();
        calibrationPoints.add(new CalibrationPoints(84.0,  1900.0,  0.55));
        calibrationPoints.add(new CalibrationPoints(58.0,  1785.0,  0.5));
        calibrationPoints.add(new CalibrationPoints(100.0,  2160.0,  0.565));
        calibrationPoints.add(new CalibrationPoints(125.0,  2300,  0.572));
        calibrationPoints.add(new CalibrationPoints(145.0,  2350,  0.5));
        calibrationPoints.sort(Comparator.comparingDouble(a -> a.distanceToGoal));
    }
    private void CalculateShooterParameters() {
        double distance = getDistanceToGoal();

        // --- Velocity compensation for shooting on the move ---
        double velX = robot.pinpointDriver.getVelX(DistanceUnit.INCH);
        double velY = robot.pinpointDriver.getVelY(DistanceUnit.INCH);

        double dx = getGoalX() - getFieldX();
        double dy = getGoalY() - getFieldY();

        double ux = dx / distance;
        double uy = dy / distance;

        double approachVelocity = velX * ux + velY * uy;
        double flightTime = 0;
        if (dynamicTargetFlyWheelVelocity > 0) {
            flightTime = distance / (dynamicTargetFlyWheelVelocity * rConstants.ShooterConstants.flywheelToBallSpeedRatio);
        }

        double compensatedDistance = distance - (approachVelocity * flightTime);
        compensatedDistance = Math.max(0, compensatedDistance);

        // --- Calibration point interpolation ---
        if (calibrationPoints.isEmpty()) {
            dynamicTargetFlyWheelVelocity = 0.0;
            dynamicTargetHoodPosition     = 0.0;
            return;
        }

        if (compensatedDistance <= calibrationPoints.get(0).distanceToGoal) {
            dynamicTargetFlyWheelVelocity = calibrationPoints.get(0).flyWheelVelocity;
            dynamicTargetHoodPosition     = calibrationPoints.get(0).hoodPosition;
            return;
        }

        if (compensatedDistance >= calibrationPoints.get(calibrationPoints.size() - 1).distanceToGoal) {
            CalibrationPoints last = calibrationPoints.get(calibrationPoints.size() - 1);
            dynamicTargetFlyWheelVelocity = last.flyWheelVelocity;
            dynamicTargetHoodPosition     = last.hoodPosition;
            return;
        }

        for (int i = 0; i < calibrationPoints.size() - 1; i++) {
            CalibrationPoints p1 = calibrationPoints.get(i);
            CalibrationPoints p2 = calibrationPoints.get(i + 1);

            if (compensatedDistance >= p1.distanceToGoal && compensatedDistance <= p2.distanceToGoal) {
                double t = (compensatedDistance - p1.distanceToGoal) / (p2.distanceToGoal - p1.distanceToGoal);

                dynamicTargetFlyWheelVelocity = p1.flyWheelVelocity + t * (p2.flyWheelVelocity - p1.flyWheelVelocity);
                dynamicTargetHoodPosition = p1.hoodPosition + t * (p2.hoodPosition - p1.hoodPosition);

                dynamicTargetFlyWheelVelocity = Math.max(0, Math.min(rConstants.ShooterConstants.maximumFlyWheelVelocity, dynamicTargetFlyWheelVelocity));
                dynamicTargetHoodPosition = Math.max(rConstants.ShooterConstants.minimumHoodPosition, Math.min(rConstants.ShooterConstants.maximumHoodPosition, dynamicTargetHoodPosition));
                return;
            }
        }
    }







    private void TelemetryUpdating(){
        telemetry.addData("Selected Drive Mode: ", rConstants.Enums.selectedDriveMode);

        telemetry.addData("X Pose: ",  "%.1f" , getFieldX());
        telemetry.addData("Y Pose: ",  "%.1f" , getFieldY());
        telemetry.addData("Distance To Goal: ", getDistanceToGoal());
        telemetry.addData("X Velocity: ", "%.2f" , getXVelocity());
        telemetry.addData("Y Velocity: ", "%.2f" , getYVelocity());
        telemetry.addData("Current Heading: ", "%.1f" , Math.toDegrees(getHeading()));
        telemetry.addData("Angle To Goal: ", "%.1f" , getAngleToGoal());

        telemetry.addData("Intake State: ", intakeSubsystem.getState());
        telemetry.addData("Intake Velocity: ", "%.0f" , intakeSubsystem.getIntakeVelocity());
        telemetry.addData("Intake Current Draw: ", "%.1f",  intakeSubsystem.getIntakeCurrentDraw());

        telemetry.addData("Spindexer State: ", spindexerSubsystem.getSpindexerState());
        telemetry.addData("Spindexer Velocity: ", spindexerSubsystem.getVelocity());
        telemetry.addData("Spindexer Position: ", spindexerSubsystem.getPosition());
        telemetry.addData("Spindexer Position Reached?: ", spindexerSubsystem.spindexerPositionReached());

        telemetry.addData("Current FlyWheel State: ", rConstants.Enums.currentShooterState);
        telemetry.addData("Current FlyWheel Velocity: ", "%.0f", shooterSubsystem.getCurrentFlyWheelVelocity());
        telemetry.addData("Target FlyWheel Wheel Velocity: ", "%.0f", dynamicTargetFlyWheelVelocity);
        telemetry.addData("Target Hood Position: ", "%.0f", dynamicTargetHoodPosition);

        telemetry.addData("Loop Time: ", elapsedTime.milliseconds());
        elapsedTime.reset();

        telemetry.update();
    }





    private void RumbleGamePad(int duration){
        gamepad1.rumble(1.0, 1.0, duration);
    }




    //Helpers
    private double getHeading() { return robot.pinpointDriver.getHeading(AngleUnit.DEGREES); }
    private double getXPose() { return robot.pinpointDriver.getEncoderX(); }
    private double getYPose() { return robot.pinpointDriver.getEncoderY(); }
    private double getFieldHeading() {
        if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE)
            headingOffset = rConstants.AllianceHeadingOffsets.blueAllianceHeadingOffset;
        else if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.RED)
            headingOffset = rConstants.AllianceHeadingOffsets.redAllianceHeadingOffset;

        return getHeading() - headingOffset;
    }

    private double getXVelocity() { return robot.pinpointDriver.getVelX(); }
    private double getYVelocity() { return robot.pinpointDriver.getVelY(); }

    private double getFieldX() { return rConstants.FieldConstants.startingXPosition + robot.pinpointDriver.getPosX(DistanceUnit.INCH); }
    private double getFieldY() { return rConstants.FieldConstants.startingYPosition + robot.pinpointDriver.getPosY(DistanceUnit.INCH); }

    private double getDistanceToGoal(){
        double goalX, goalY;



        if (rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE) {
            goalX = rConstants.FieldConstants.blueGoalXPosition;
            goalY = rConstants.FieldConstants.blueGoalYPosition;
        } else {
            goalX = rConstants.FieldConstants.redGoalXPosition;
            goalY = rConstants.FieldConstants.redGoalYPosition;
        }


        double dx = goalX - getFieldX();
        double dy = goalY - getFieldY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    private double getAngleToGoal() {
        double goalX, goalY;

        if (rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE) {
            goalX = rConstants.FieldConstants.blueGoalXPosition;
            goalY = rConstants.FieldConstants.blueGoalYPosition;
        } else {
            goalX = rConstants.FieldConstants.redGoalXPosition;
            goalY = rConstants.FieldConstants.redGoalYPosition;
        }

        double dx = goalX - getFieldX();
        double dy = goalY - getFieldY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }



    private double getGoalX() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalXPosition
                : rConstants.FieldConstants.redGoalXPosition;
    }
    private double getGoalY() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalYPosition
                : rConstants.FieldConstants.redGoalYPosition;
    }
}