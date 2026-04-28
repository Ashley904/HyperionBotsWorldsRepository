package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.InitializeTransferCMD;
import org.firstinspires.ftc.teamcode.commands.ShootArtefactsCMD;
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
    private ButtonReader autoLockHeadingButtonReader;
    private ButtonReader resetPoseButtonReader;





    private int loopCount = 0;
    private final ElapsedTime elapsedTime = new ElapsedTime();





    private static final List<CalibrationPoints> calibrationPoints = new ArrayList<>();
    double dynamicTargetFlyWheelVelocity = 0.0, dynamicTargetHoodPosition = 0.0;






    //----------Precomputed Shooter Lookup Table----------//
    private static final double LOOKUP_TABLE_RESOLUTION = 0.01;
    private static final double LOOKUP_TABLE_MAX_DISTANCE = 200.0;
    private static final int LOOKUP_TABLE_SIZE = (int) (LOOKUP_TABLE_MAX_DISTANCE / LOOKUP_TABLE_RESOLUTION) + 1;
    private final double[] precomputedFlyWheelVelocities = new double[LOOKUP_TABLE_SIZE];
    private final double[] precomputedHoodPositions = new double[LOOKUP_TABLE_SIZE];
    //----------end----------//





    int currentSpindexerIndex = 0;





    //----------Limelight Auto-Lock Heading----------//
    private Limelight3A limelight;
    private boolean autoLockHeadingEnabled = false;

    private double limelightLastError = 0;
    private boolean limelightHasLastError = false;
    private double limelightLastTime = 0;
    private final ElapsedTime limelightRuntime = new ElapsedTime();
    //----------end----------//






    @Override
    public void init(){
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);





        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(rConstants.FieldConstants.autonomousEndPose);





        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());





        // Enabling Bulk Reading
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }





        turretSubsystem = new TurretSubsystem(robot);
        intakeSubsystem = new IntakeSubsystem(robot);
        shooterSubsystem = new ShooterSubsystem(robot);
        spindexerSubsystem = new SpindexerSubsystem(robot);
        spindexerSensorsSubsystem = new SpindexerSensorsSubsystem(robot);





        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();





        //Calling Functions
        InitializeGamePadControls();
        InitializeCalibrationPoints();
        PrecomputeShooterLookupTable();




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

        limelight.start();
        limelightRuntime.reset();
    }





    @Override
    public void loop(){
        follower.update();

        GamepadControlsManaging();
        BackgroundOperations();
        RobotDrive();
        CalculateShooterParameters();

        if(loopCount++ % 5 == 0) TelemetryUpdating();
    }





    private void RobotDrive(){
        double adjustedDrivingSpeed = rConstants.DriveTrainConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + rConstants.DriveTrainConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, rConstants.DriveTrainConstants.minimumDriveTrainSpeed);


        double y = -gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x = gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = gamepad1.right_stick_x * adjustedDrivingSpeed;


        //----------Field-Centric Rotation----------//
        double heading = getHeading();
        double headingOffset = rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? Math.toRadians(rConstants.AllianceHeadingOffsets.blueAllianceHeadingOffset)
                : Math.toRadians(rConstants.AllianceHeadingOffsets.redAllianceHeadingOffset);

        double rotationAngle = -heading + headingOffset;
        double rotatedX = x * Math.cos(rotationAngle) - y * Math.sin(rotationAngle);
        double rotatedY = x * Math.sin(rotationAngle) + y * Math.cos(rotationAngle);
        //----------end----------//




        //----------Limelight Auto-Lock Heading----------//
        if(autoLockHeadingEnabled) {
            double currentTime = limelightRuntime.seconds();
            double deltaTime = currentTime - limelightLastTime;
            limelightLastTime = currentTime;

            LLResult llResult = limelight.getLatestResult();

            if(llResult == null || !llResult.isValid()) {
                limelightHasLastError = false;
            } else {
                // Scale the targeting offset down as distance increases.
                // Full offset at close range, scaled-down offset at/beyond far distance.
                double distance = getDistanceToGoal();
                double t = Math.max(0, Math.min(1, distance / rConstants.LimelightAutoAimConstants.targetingOffsetFarDistance));
                double offsetScalar = 1.0 - t * (1.0 - rConstants.LimelightAutoAimConstants.targetingOffsetFarScalar);
                double scaledOffset = rConstants.LimelightAutoAimConstants.targetingOffset * offsetScalar;

                double error = llResult.getTx() - scaledOffset;

                double limelightTurnPower;
                if(Math.abs(error) > rConstants.LimelightAutoAimConstants.limelightHeadingTolerance) {
                    double pTerm = rConstants.LimelightAutoAimConstants.limelightKp * error;

                    double dTerm = 0;
                    if(limelightHasLastError && deltaTime > 1e-4) {
                        dTerm = rConstants.LimelightAutoAimConstants.limelightKd * ((error - limelightLastError) / deltaTime);
                    }

                    limelightTurnPower = pTerm + dTerm;
                } else {
                    limelightTurnPower = 0;
                }

                limelightTurnPower *= rConstants.LimelightAutoAimConstants.limelightTurnDirection;
                limelightTurnPower = Math.max(-rConstants.LimelightAutoAimConstants.maxCorrectionPower,
                        Math.min(rConstants.LimelightAutoAimConstants.maxCorrectionPower, limelightTurnPower));

                rx = limelightTurnPower;

                limelightLastError = error;
                limelightHasLastError = true;
            }
        } else {
            limelightHasLastError = false;
            limelightLastTime = limelightRuntime.seconds();
        }
        //----------end----------//




        double frontLeftMotorPower  = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y + x + rx) : (rotatedY + rotatedX + rx);
        double backLeftMotorPower   = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y - x + rx) : (rotatedY - rotatedX + rx);
        double frontRightMotorPower = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y - x - rx) : (rotatedY - rotatedX - rx);
        double backRightMotorPower  = rConstants.Enums.selectedDriveMode == rConstants.Enums.DriveMode.RobotCentric ? (y + x - rx) : (rotatedY + rotatedX - rx);


        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftMotorPower), Math.abs(backLeftMotorPower)),
                Math.max(Math.abs(frontRightMotorPower), Math.abs(backRightMotorPower))
        );


        if(maxPower > rConstants.DriveTrainConstants.maximumDriveTrainSpeed){
            frontLeftMotorPower  /= maxPower;
            backLeftMotorPower   /= maxPower;
            frontRightMotorPower /= maxPower;
            backRightMotorPower  /= maxPower;
        }


        robot.front_left_motor.setPower(frontLeftMotorPower);
        robot.back_left_motor.setPower(backLeftMotorPower);
        robot.front_right_motor.setPower(frontRightMotorPower);
        robot.back_right_motor.setPower(backRightMotorPower);
    }





    private void GamepadControlsManaging(){
        //----------Intaking Managing----v------//
        if(spindexerSubsystem.getSpindexerState() != SpindexerSubsystem.SpindexerState.Shooting){
            enableIntakeButtonReader.readValue();
            reverseIntakeButtonReader.readValue();

            if(enableIntakeButtonReader.wasJustPressed()){
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


            if(reverseIntakeButtonReader.wasJustPressed()){
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
        if(shootArtefactsButtonReader.wasJustPressed()) {
            shootArtefactsCMD = new ShootArtefactsCMD(spindexerSubsystem, intakeSubsystem, robot);
            CommandScheduler.getInstance().schedule(shootArtefactsCMD);
        }
        //----------end----------//





        //----------Toggle Auto-Lock Heading----------//
        autoLockHeadingButtonReader.readValue();
        if(autoLockHeadingButtonReader.wasJustPressed()) {
            autoLockHeadingEnabled = !autoLockHeadingEnabled;
            limelightHasLastError = false;
            limelightLastTime = limelightRuntime.seconds();
            RumbleGamePad(100);
        }
        //----------end----------//





        //----------Reset Pose----------//
        resetPoseButtonReader.readValue();
        if(resetPoseButtonReader.wasJustPressed()) {
            follower.setPose(rConstants.FieldConstants.resetPose);
            RumbleGamePad(250);
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
        indexSpindexerButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.indexSpindexer);
        selectAllianceButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, rConstants.GamePadControls.selectAlliance);
        autoLockHeadingButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, GamepadKeys.Button.LEFT_BUMPER);
        resetPoseButtonReader = new ButtonReader(rConstants.GamePadControls.gamepad1EX, GamepadKeys.Button.START);
    }





    private void BackgroundOperations(){
        intakeSubsystem.periodic();
        turretSubsystem.periodic();
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();

        shooterSubsystem.setTargetVelocity(dynamicTargetFlyWheelVelocity);
        shooterSubsystem.setHoodPosition(dynamicTargetHoodPosition);

        turretSubsystem.setTurretAngle(0);

        SpindexerManaging();
        CommandScheduler.getInstance().run();
    }
    private void SpindexerManaging(){
        if(spindexerSubsystem.getSpindexerState() == SpindexerSubsystem.SpindexerState.Shooting
                || intakeSubsystem.getState() != IntakeSubsystem.IntakeState.Intaking
                || !spindexerSubsystem.spindexerPositionReached()) return;
        if(loopCount % 3 != 0) return;

        double leftDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
        boolean ballPresent = leftDistance <= rConstants.SensorConstants.distanceSensorOccupiedThreshold;

        if(ballPresent){
            currentSpindexerIndex = (currentSpindexerIndex + 1) % rConstants.SpindexerConstants.intakingPositions.length;
            spindexerSubsystem.setSpindexerPosition(
                    rConstants.SpindexerConstants.intakingPositions[currentSpindexerIndex]
            );
        }
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

        calibrationPoints.add(new CalibrationPoints(44.0,  1065.0,  0.69));
        calibrationPoints.add(new CalibrationPoints(80.0,  1180,  0.575));
        calibrationPoints.add(new CalibrationPoints(105.0,  1320,  0.56));
        calibrationPoints.add(new CalibrationPoints(121.0,  1405,  0.54));
        calibrationPoints.add(new CalibrationPoints(141.0,  1485.0,  0.52));
        calibrationPoints.sort(Comparator.comparingDouble(a -> a.distanceToGoal));
    }
    private void PrecomputeShooterLookupTable() {
        // Build the lookup table once at init by running the same interpolation logic
        for (int i = 0; i < LOOKUP_TABLE_SIZE; i++) {
            double distance = i * LOOKUP_TABLE_RESOLUTION;

            if (calibrationPoints.isEmpty()) {
                precomputedFlyWheelVelocities[i] = 0.0;
                precomputedHoodPositions[i] = 0.0;
                continue;
            }

            // Below the lowest calibration point: clamp to first point
            if (distance <= calibrationPoints.get(0).distanceToGoal) {
                precomputedFlyWheelVelocities[i] = calibrationPoints.get(0).flyWheelVelocity;
                precomputedHoodPositions[i] = calibrationPoints.get(0).hoodPosition;
                continue;
            }

            // Above the highest calibration point: clamp to last point
            CalibrationPoints last = calibrationPoints.get(calibrationPoints.size() - 1);
            if (distance >= last.distanceToGoal) {
                precomputedFlyWheelVelocities[i] = last.flyWheelVelocity;
                precomputedHoodPositions[i] = last.hoodPosition;
                continue;
            }

            // In between: linear interpolation between the two surrounding points
            for (int j = 0; j < calibrationPoints.size() - 1; j++) {
                CalibrationPoints p1 = calibrationPoints.get(j);
                CalibrationPoints p2 = calibrationPoints.get(j + 1);

                if (distance >= p1.distanceToGoal && distance <= p2.distanceToGoal) {
                    double t = (distance - p1.distanceToGoal) / (p2.distanceToGoal - p1.distanceToGoal);

                    double flyWheelVel = p1.flyWheelVelocity + t * (p2.flyWheelVelocity - p1.flyWheelVelocity);
                    double hoodPos = p1.hoodPosition + t * (p2.hoodPosition - p1.hoodPosition);

                    // Clamp to allowed ranges
                    flyWheelVel = Math.max(0, Math.min(rConstants.ShooterConstants.maximumFlyWheelVelocity, flyWheelVel));
                    hoodPos = Math.max(rConstants.ShooterConstants.minimumHoodPosition, Math.min(rConstants.ShooterConstants.maximumHoodPosition, hoodPos));

                    precomputedFlyWheelVelocities[i] = flyWheelVel;
                    precomputedHoodPositions[i] = hoodPos;
                    break;
                }
            }
        }
    }
    private void CalculateShooterParameters() {
        double distance = getDistanceToGoal();

        // Fast O(1) lookup instead of looping through calibration points and interpolating
        int index = (int) (distance / LOOKUP_TABLE_RESOLUTION);
        if (index < 0) index = 0;
        if (index >= LOOKUP_TABLE_SIZE) index = LOOKUP_TABLE_SIZE - 1;

        dynamicTargetFlyWheelVelocity = precomputedFlyWheelVelocities[index];
        dynamicTargetHoodPosition = precomputedHoodPositions[index];
    }







    private void TelemetryUpdating(){
        telemetry.addData("Selected Drive Mode: ", rConstants.Enums.selectedDriveMode);

        telemetry.addData("X Pose: ",  "%.1f" , getXPose());
        telemetry.addData("Y Pose: ",  "%.1f" , getYPose());
        telemetry.addData("Distance To Goal: ", "%.1f", getDistanceToGoal());
        telemetry.addData("Current Heading: ", "%.1f" , Math.toDegrees(getHeading()));

        telemetry.addData("Intake State: ", intakeSubsystem.getState());

        telemetry.addData("Spindexer State: ", spindexerSubsystem.getSpindexerState());
        telemetry.addData("Spindexer Velocity: ", spindexerSubsystem.getVelocity());
        telemetry.addData("Spindexer Position: ", spindexerSubsystem.getPosition());
        telemetry.addData("Spindexer Position Reached?: ", spindexerSubsystem.spindexerPositionReached());

        telemetry.addData("Current FlyWheel Velocity: ", "%.0f", shooterSubsystem.getCurrentFlyWheelVelocity());
        telemetry.addData("Target FlyWheel Wheel Velocity: ", "%.0f", dynamicTargetFlyWheelVelocity);

        telemetry.addData("Auto-Lock Heading: ", autoLockHeadingEnabled);
        telemetry.addData("Tag Visible: ", limelight.getLatestResult() != null && limelight.getLatestResult().isValid());
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





    private double getDistanceToGoal() {
        return follower.getPose().distanceFrom(getActiveGoalPose());
    }
}