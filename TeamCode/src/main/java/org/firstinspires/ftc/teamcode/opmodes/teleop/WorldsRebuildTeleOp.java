package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.telephony.emergency.EmergencyNumber;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.InitializeTransferCMD;
import org.firstinspires.ftc.teamcode.commands.ShootArtefactsCMD;
import org.firstinspires.ftc.teamcode.controllers.PDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
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
    Follower follower;





    IntakeSubsystem intakeSubsystem;
    TurretSubsystem turretSubsystem;
    ShooterSubsystem shooterSubsystem;
    SpindexerSubsystem spindexerSubsystem;
    SpindexerSensorsSubsystem spindexerSensorsSubsystem;





    ShootArtefactsCMD shootArtefactsCMD;





    private RobotHardwareMap robot;





    private ButtonReader cycleDriveModesButtonReader;
    private ButtonReader enableIntakeButtonReader, reverseIntakeButtonReader;
    private ButtonReader shootArtefactsButtonReader;
    private ButtonReader indexSpindexerButtonReader;

    private ButtonReader selectAllianceButtonReader;





    private double targetHeading=0.0;
    private boolean headingCorrectionEnabled=true;





    private int loopCount = 0;
    private final ElapsedTime elapsedTime = new ElapsedTime();





    private static final List<CalibrationPoints> calibrationPoints = new ArrayList<>();
    double dynamicTargetFlyWheelVelocity = 0.0, dynamicTargetHoodPosition = 0.0;





    int currentSpindexerIndex = 0;
    double turretAngle = 0.0;
    double offset = 0.0;






    @Override
    public void init(){
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        robot.pinpointDriver.recalibrateIMU();





        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(rConstants.FieldConstants.startingPose);





        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());





        // Enabling Bulk Reading
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }





        intakeSubsystem = new IntakeSubsystem(robot);
        turretSubsystem = new TurretSubsystem(robot);
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






        telemetry.addData("Status: ", "Initialization Complete...");
        telemetry.update();
    }





    @Override
    public void init_loop(){
        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();

        spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[0]);
        CommandScheduler.getInstance().run();




        // Selecting Alliance
        selectAllianceButtonReader.readValue();
        if(selectAllianceButtonReader.wasJustPressed()){
            if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.RED; }
            else if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.RED) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.BLUE; }
        }




        telemetry.addData("Selected Alliance: ", rConstants.Enums.selectedAlliance);
        telemetry.addData("Cycle Alliance: ", "A Button");
        telemetry.update();
    }





    @Override
    public void start() {
        CommandScheduler.getInstance().reset();

        robot.spindexerEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.spindexerEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[1]);
    }





    @Override
    public void loop(){
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
        follower.update();
        follower.startTeleOpDrive();

        GamepadControlsManaging();
        BackgroundOperations();
        RobotDrive();
        CalculateShooterParameters();
        SpindexerManaging();

        if(loopCount++ % 3 == 0) TelemetryUpdating();
    }





    private void RobotDrive(){
        double adjustedDrivingSpeed = rConstants.DriveTrainConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + rConstants.DriveTrainConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, rConstants.DriveTrainConstants.minimumDriveTrainSpeed);

        double y = -gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x = -gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = -gamepad1.right_stick_x * adjustedDrivingSpeed;

        if(rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric){
            follower.setTeleOpDrive(
                    y,
                    x,
                    rx,
                    true
            );
        }else if (rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.FieldCentric){
            if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE) offset = rConstants.AllianceHeadingOffsets.blueAllianceHeadingOffset;
            else if(rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.RED) offset = rConstants.AllianceHeadingOffsets.redAllianceHeadingOffset;

            double cos = Math.cos(offset);
            double sin = Math.sin(offset);

            double rotatedX = x * cos - y * sin;
            double rotatedY = x * sin + y * cos;

            follower.setTeleOpDrive(
                    rotatedY,
                    rotatedX,
                    rx,
                    false
            );
        }
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

                spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[0]);
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
        selectAllianceButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.selectAlliance);
    }





    private void BackgroundOperations(){
        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();


        shooterSubsystem.setTargetVelocity(1240);
        shooterSubsystem.setHoodPosition(0.54);


        robot.pinpointDriver.update();
        CommandScheduler.getInstance().run();
    }
    private void SpindexerManaging(){
        if(spindexerSubsystem.getSpindexerState() == SpindexerSubsystem.SpindexerState.Shooting
                || intakeSubsystem.getState() != IntakeSubsystem.IntakeState.Intaking
                || !spindexerSubsystem.spindexerPositionReached()) return;

        double leftDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
        double rightDistance = robot.rightDistanceSensor.getDistance(DistanceUnit.CM);

        boolean ballPresent = leftDistance <= rConstants.SensorConstants.distanceSensorOccupiedThreshold
                || rightDistance <= rConstants.SensorConstants.distanceSensorOccupiedThreshold;

        if(ballPresent){
            currentSpindexerIndex = (currentSpindexerIndex + 1) % rConstants.SpindexerConstants.intakingPositions.length;
            spindexerSubsystem.setSpindexerPosition(
                    rConstants.SpindexerConstants.intakingPositions[currentSpindexerIndex]
            );
        }
    }





    private double calculateTurretAngle() {
        Pose goalPose = getActiveGoalPose();
        return Math.atan2(goalPose.getY() - getYPose(), goalPose.getX() - getXPose());
    }
    private Pose getActiveGoalPose() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalPose
                : rConstants.FieldConstants.redGoalPose;
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
        calibrationPoints.add(new CalibrationPoints(87.0,  1200,  0.6));
        calibrationPoints.add(new CalibrationPoints(57.6,  1000,  0.6));
        calibrationPoints.add(new CalibrationPoints(93.9,  1240,  0.54));
    }
    private void CalculateShooterParameters() {
        double distance = getDistanceToGoal();

        double velX = follower.getVelocity().getYComponent();
        double velY = follower.getVelocity().getXComponent();

        double dx = getGoalX() - getXPose();
        double dy = getGoalY() - getYPose();

        double ux = dx / distance;
        double uy = dy / distance;

        double approachVelocity = velX * ux + velY * uy;
        double flightTime = 0;
        if (dynamicTargetFlyWheelVelocity > 0) {
            flightTime = distance / (dynamicTargetFlyWheelVelocity * rConstants.ShooterConstants.flywheelToBallSpeedRatio);
        }

        double compensatedDistance = distance - (approachVelocity * flightTime);
        compensatedDistance = Math.max(0, compensatedDistance);

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

        telemetry.addData("X Pose: ",  "%.1f" , getXPose());
        telemetry.addData("Y Pose: ",  "%.1f" , getYPose());
        telemetry.addData("Distance To Goal: ", "%.1f", getDistanceToGoal());
        telemetry.addData("Current Heading: ", "%.1f" , Math.toDegrees(getHeading()));

        telemetry.addData("Intake State: ", intakeSubsystem.getState());
        telemetry.addData("Intake Velocity: ", "%.0f" , intakeSubsystem.getIntakeVelocity());

        telemetry.addData("Current FlyWheel Velocity: ", "%.0f", shooterSubsystem.getCurrentFlyWheelVelocity());
        telemetry.addData("Target FlyWheel Wheel Velocity: ", "%.0f", dynamicTargetFlyWheelVelocity);

        telemetry.addData("Turret Target Angle: ", "%.1f", calculateTurretAngle());

        telemetry.update();
    }





    private void RumbleGamePad(int duration){
        gamepad1.rumble(1.0, 1.0, duration);
    }





    //Helpers
    private double getHeading() { return follower.getPose().getHeading(); }
    private double getXPose() { return follower.getPose().getX(); }
    private double getYPose() { return follower.getPose().getY(); }





    private double getDistanceToGoal() {
        double goalX = getGoalX();
        double goalY = getGoalY();

        double dx = goalX - getXPose();
        double dy = goalY - getYPose();

        return Math.sqrt(dx * dx + dy * dy);
    }





    private double getGoalX() {
        return getActiveGoalPose().getX();
    }
    private double getGoalY() {
        return getActiveGoalPose().getY();
    }
}