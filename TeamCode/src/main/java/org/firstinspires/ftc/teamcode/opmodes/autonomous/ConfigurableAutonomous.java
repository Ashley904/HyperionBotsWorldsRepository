package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.InitializeTransferCMD;
import org.firstinspires.ftc.teamcode.commands.ShootArtefactsCMD;
import org.firstinspires.ftc.teamcode.customPathing.Point;
import org.firstinspires.ftc.teamcode.opmodes.teleop.WorldsRebuildTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Config
@Autonomous(name = "Worlds Configurable Autonomous", group = "Autonomous")
public class ConfigurableAutonomous extends OpMode {

    private enum Phase { SelectingAlliance, SelectingStartingZone, ConstructingActions, Confirming, Ready }
    private enum ActionType { ShootCloseZone, ShootFarZone, CollectCloseSet, CollectMiddleSet, CollectFarSet, GateCollect }

    public static double collectSlowSpeed = 0.6;


    private RobotHardwareMap robot;
    private Follower follower;


    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private SpindexerSensorsSubsystem spindexerSensorsSubsystem;


    private Phase currentPhase = Phase.SelectingAlliance;
    private boolean isCloseZone = true;
    private List<ActionType> actionQueue = new ArrayList<>();
    private Pose trackingPose;


    private ButtonReader selectBlueAllianceButtonReader, selectRedAllianceButtonReader;
    private ButtonReader selectCloseZoneButtonReader, selectFarZoneButtonReader;
    private ButtonReader shootCloseZoneButtonReader, shootFarZoneButtonReader;
    private ButtonReader collectCloseSetButtonReader, collectMiddleSetButtonReader, collectFarSetButtonReader;
    private ButtonReader gateCollectButtonReader;
    private ButtonReader undoButtonReader, confirmButtonReader;


    int currentSpindexerIndex = 0;


    private static final List<CalibrationPoints> calibrationPoints = new ArrayList<>();
    double dynamicTargetFlyWheelVelocity = 0.0, dynamicTargetHoodPosition = 0.0;


    // ═══════════════════════════════════════════
    //  FOLLOW PATH COMMAND
    // ═══════════════════════════════════════════

    private class FollowPathCommand extends CommandBase {
        private final Pose fromPose;
        private final Pose toPose;
        private final double maxSpeed;

        public FollowPathCommand(Pose fromPose, Pose toPose) {
            this.fromPose = fromPose;
            this.toPose = toPose;
            this.maxSpeed = 1.0;
        }

        public FollowPathCommand(Pose fromPose, Pose toPose, double maxSpeed) {
            this.fromPose = fromPose;
            this.toPose = toPose;
            this.maxSpeed = maxSpeed;
        }

        @Override
        public void initialize() {
            PathChain pathChain = follower.pathBuilder()
                    .addPath(new BezierLine(fromPose, toPose))
                    .setLinearHeadingInterpolation(fromPose.getHeading(), toPose.getHeading())
                    .build();
            follower.followPath(pathChain, maxSpeed, false);
        }

        @Override public void execute() {}
        @Override public boolean isFinished() { return !follower.isBusy(); }
        @Override public void end(boolean interrupted) {}
    }


    // ═══════════════════════════════════════════
    //  OP MODE METHODS
    // ═══════════════════════════════════════════

    @Override
    public void init() {
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        robot.pinpointDriver.recalibrateIMU();

        follower = Constants.createFollower(hardwareMap);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); }

        intakeSubsystem = new IntakeSubsystem(robot);
        turretSubsystem = new TurretSubsystem(robot);
        shooterSubsystem = new ShooterSubsystem(robot);
        spindexerSubsystem = new SpindexerSubsystem(robot);
        spindexerSensorsSubsystem = new SpindexerSensorsSubsystem(robot);

        intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled);
        shooterSubsystem.setHoodPosition(rConstants.ShooterConstants.minimumHoodPosition);
        shooterSubsystem.setTargetVelocity(0);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().schedule(new InitializeTransferCMD(robot));
        InitializeButtonReaders();
        InitializeCalibrationPoints();

        telemetry.addData("Status", "Initialization Complete...");
        telemetry.update();
    }


    @Override
    public void init_loop() {
        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();

        spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[0]);
        CommandScheduler.getInstance().run();

        ReadAllButtons();

        switch (currentPhase) {
            case SelectingAlliance:     HandleAllianceSelection();  break;
            case SelectingStartingZone: HandleZoneSelection();      break;
            case ConstructingActions:   HandleActionBuilding();     break;
            case Confirming:            HandleConfirmation();       break;
            case Ready: break;
        }

        DisplayStatus();
    }


    @Override
    public void start() {
        CommandScheduler.getInstance().reset();

        robot.spindexerEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.spindexerEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        boolean isBlue = rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE;
        Pose startPose;
        if (isCloseZone) {
            startPose = pointToPose(isBlue
                    ? rConstants.AutonomousPositionConstants.blueCloseZoneStartingPosition
                    : rConstants.AutonomousPositionConstants.redCloseZoneStartingPosition);
        } else {
            startPose = pointToPose(isBlue
                    ? rConstants.AutonomousPositionConstants.blueCloseZoneStartingPosition
                    : rConstants.AutonomousPositionConstants.redCloseZoneStartingPosition);
        }
        follower.setStartingPose(startPose);
        trackingPose = startPose;

        spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[0]);
        CommandScheduler.getInstance().schedule(BuildAutoSequence());
    }


    @Override
    public void loop() {
        follower.update();

        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        turretSubsystem.periodic();
        spindexerSubsystem.periodic();

        // Calculate shooter parameters dynamically based on distance to goal
        CalculateShooterParameters();

        shooterSubsystem.setTargetVelocity(dynamicTargetFlyWheelVelocity);
        shooterSubsystem.setHoodPosition(dynamicTargetHoodPosition);
        turretSubsystem.setTurretAngle(0);

        // Auto-indexing runs every loop while intake is on
        SpindexerManaging();
        CommandScheduler.getInstance().run();

        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Distance To Goal", "%.1f", getDistanceToGoal());
        telemetry.addData("Target Velocity", "%.0f", dynamicTargetFlyWheelVelocity);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Intake State", intakeSubsystem.getState());
        telemetry.addData("Spindexer State", spindexerSubsystem.getSpindexerState());
        telemetry.update();
    }


    @Override
    public void stop() {
        rConstants.FieldConstants.startingPose = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
        );
        CommandScheduler.getInstance().reset();
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


    // ═══════════════════════════════════════════
    //  BUTTON READERS
    // ═══════════════════════════════════════════

    private void InitializeButtonReaders() {
        GamepadEx gamepad1EX = new GamepadEx(gamepad1);

        selectBlueAllianceButtonReader  = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectBlueAlliance);
        selectRedAllianceButtonReader   = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectRedAlliance);
        selectCloseZoneButtonReader     = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectCloseZoneStartingPosition);
        selectFarZoneButtonReader       = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectFarZoneStartingPosition);
        shootCloseZoneButtonReader      = new ButtonReader(gamepad1EX, rConstants.GamePadControls.shootCloseZoneMapping);
        shootFarZoneButtonReader        = new ButtonReader(gamepad1EX, rConstants.GamePadControls.shootFarZoneMapping);
        collectCloseSetButtonReader     = new ButtonReader(gamepad1EX, rConstants.GamePadControls.collectCloseSetMapping);
        collectMiddleSetButtonReader    = new ButtonReader(gamepad1EX, rConstants.GamePadControls.collectMiddleSetMapping);
        collectFarSetButtonReader       = new ButtonReader(gamepad1EX, rConstants.GamePadControls.collectFarSetMapping);
        gateCollectButtonReader         = new ButtonReader(gamepad1EX, rConstants.GamePadControls.gateCollectMapping);
        undoButtonReader                = new ButtonReader(gamepad1EX, GamepadKeys.Button.BACK);
        confirmButtonReader             = new ButtonReader(gamepad1EX, GamepadKeys.Button.START);
    }

    private void ReadAllButtons() {
        selectBlueAllianceButtonReader.readValue();  selectRedAllianceButtonReader.readValue();
        selectCloseZoneButtonReader.readValue();     selectFarZoneButtonReader.readValue();
        shootCloseZoneButtonReader.readValue();      shootFarZoneButtonReader.readValue();
        collectCloseSetButtonReader.readValue();     collectMiddleSetButtonReader.readValue();
        collectFarSetButtonReader.readValue();       gateCollectButtonReader.readValue();
        undoButtonReader.readValue();                confirmButtonReader.readValue();
    }


    // ═══════════════════════════════════════════
    //  INIT LOOP HANDLERS
    // ═══════════════════════════════════════════

    private void HandleAllianceSelection() {
        if (selectBlueAllianceButtonReader.wasJustPressed()) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.BLUE; currentPhase = Phase.SelectingStartingZone; }
        if (selectRedAllianceButtonReader.wasJustPressed()) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.RED; currentPhase = Phase.SelectingStartingZone; }
    }

    private void HandleZoneSelection() {
        if (selectCloseZoneButtonReader.wasJustPressed()) {
            isCloseZone = true;
            actionQueue.add(ActionType.ShootCloseZone);
            currentPhase = Phase.ConstructingActions;
        }
        if (selectFarZoneButtonReader.wasJustPressed()) {
            isCloseZone = false;
            actionQueue.add(ActionType.ShootFarZone);
            currentPhase = Phase.ConstructingActions;
        }
    }

    private void HandleActionBuilding() {
        if (shootCloseZoneButtonReader.wasJustPressed()) actionQueue.add(ActionType.ShootCloseZone);
        if (shootFarZoneButtonReader.wasJustPressed()) actionQueue.add(ActionType.ShootFarZone);
        if (gateCollectButtonReader.wasJustPressed()) actionQueue.add(ActionType.GateCollect);
        if (collectCloseSetButtonReader.wasJustPressed()) actionQueue.add(ActionType.CollectCloseSet);
        if (collectMiddleSetButtonReader.wasJustPressed()) actionQueue.add(ActionType.CollectMiddleSet);
        if (collectFarSetButtonReader.wasJustPressed()) actionQueue.add(ActionType.CollectFarSet);

        if (undoButtonReader.wasJustPressed() && actionQueue.size() > 1) actionQueue.remove(actionQueue.size() - 1);
        if (confirmButtonReader.wasJustPressed()) currentPhase = Phase.Confirming;
    }

    private void HandleConfirmation() {
        if (confirmButtonReader.wasJustPressed()) currentPhase = Phase.Ready;
        if (undoButtonReader.wasJustPressed()) currentPhase = Phase.ConstructingActions;
    }


    // ═══════════════════════════════════════════
    //  BUILD COMMAND SEQUENCE
    // ═══════════════════════════════════════════

    private SequentialCommandGroup BuildAutoSequence() {
        List<CommandBase> commands = new ArrayList<>();
        boolean isBlue = rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE;

        for (ActionType action : actionQueue) {
            switch (action) {
                case ShootCloseZone: {
                    Pose target = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.scoreCloseZoneBlueSide
                            : rConstants.AutonomousPositionConstants.scoreCloseZoneRedSide);
                    commands.add(new FollowPathCommand(trackingPose, target));
                    commands.add(new ShootArtefactsCMD(spindexerSubsystem, intakeSubsystem, robot));
                    commands.add(new WaitUntilCommand(() ->
                            spindexerSubsystem.getSpindexerState() != SpindexerSubsystem.SpindexerState.Shooting));
                    trackingPose = target;
                    break;
                }

                case ShootFarZone: {
                    Pose target = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.scoreFarZoneBlueSide
                            : rConstants.AutonomousPositionConstants.scoreFarZoneRedSide);
                    commands.add(new FollowPathCommand(trackingPose, target));
                    commands.add(new ShootArtefactsCMD(spindexerSubsystem, intakeSubsystem, robot));
                    commands.add(new WaitUntilCommand(() ->
                            spindexerSubsystem.getSpindexerState() != SpindexerSubsystem.SpindexerState.Shooting));
                    trackingPose = target;
                    break;
                }

                case GateCollect: {
                    Pose target = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.gateCollectBlueSide
                            : rConstants.AutonomousPositionConstants.gateCollectRedSide);
                    commands.add(new FollowPathCommand(trackingPose, target));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new WaitCommand(1300));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled)));
                    trackingPose = target;
                    break;
                }

                case CollectCloseSet: {
                    Pose pose1 = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.collectThirdSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectThirdSet1RedSide);
                    Pose pose2 = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.collectThirdSet2BlueSide
                            : rConstants.AutonomousPositionConstants.collectThirdSet2RedSide);
                    commands.add(new FollowPathCommand(trackingPose, pose1));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new FollowPathCommand(pose1, pose2, collectSlowSpeed));
                    commands.add(new WaitCommand(1000));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled)));
                    trackingPose = pose2;
                    break;
                }

                case CollectMiddleSet: {
                    Pose pose1 = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.collectSecondSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectSecondSet1RedSide);
                    Pose pose2 = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.collectSecondSet2BlueSide
                            : rConstants.AutonomousPositionConstants.collectSecondSet2RedSide);
                    commands.add(new FollowPathCommand(trackingPose, pose1));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new FollowPathCommand(pose1, pose2, collectSlowSpeed));
                    commands.add(new WaitCommand(1000));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled)));
                    trackingPose = pose2;
                    break;
                }

                case CollectFarSet: {
                    Pose pose1 = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.collectFirstSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectFirstSet1RedSide);
                    Pose pose2 = pointToPose(isBlue
                            ? rConstants.AutonomousPositionConstants.collectFirstSet2BlueSide
                            : rConstants.AutonomousPositionConstants.collectFirstSet2RedSide);
                    commands.add(new FollowPathCommand(trackingPose, pose1));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new FollowPathCommand(pose1, pose2, collectSlowSpeed));
                    commands.add(new WaitCommand(1000));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled)));
                    trackingPose = pose2;
                    break;
                }
            }
        }

        return new SequentialCommandGroup(commands.toArray(new CommandBase[0]));
    }


    // ═══════════════════════════════════════════
    //  SHOOTER CALIBRATION
    // ═══════════════════════════════════════════

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
        calibrationPoints.add(new CalibrationPoints(36.0,  1110.0,  0.88));
        calibrationPoints.add(new CalibrationPoints(61.0,  1230.0,  0.88));
        calibrationPoints.add(new CalibrationPoints(88.0,  1350.0,  0.71));
        calibrationPoints.add(new CalibrationPoints(132.0,  1670,  0.75));
        calibrationPoints.add(new CalibrationPoints(150.0,  1750,  0.7));
        calibrationPoints.sort(Comparator.comparingDouble(a -> a.distanceToGoal));
    }

    private void CalculateShooterParameters() {
        double distance = getDistanceToGoal();

        double velX = follower.getVelocity().getYComponent();
        double velY = follower.getVelocity().getXComponent();

        double dx = getGoalX() - getXPose();
        double dy = getGoalY() - getYPose();

        double ux = distance == 0 ? 0 : dx / distance;
        double uy = distance == 0 ? 0 : dy / distance;

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


    // ═══════════════════════════════════════════
    //  HELPERS
    // ═══════════════════════════════════════════

    private Pose pointToPose(Point point) {
        return new Pose(point.getX(), point.getY(), Math.toRadians(point.getHeading()));
    }

    private double getHeading() { return follower.getPose().getHeading(); }
    private double getXPose() { return follower.getPose().getX(); }
    private double getYPose() { return follower.getPose().getY(); }

    private Pose getActiveGoalPose() {
        return rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE
                ? rConstants.FieldConstants.blueGoalPose
                : rConstants.FieldConstants.redGoalPose;
    }

    private double getGoalX() { return getActiveGoalPose().getX(); }
    private double getGoalY() { return getActiveGoalPose().getY(); }

    private double getDistanceToGoal() {
        double dx = getGoalX() - getXPose();
        double dy = getGoalY() - getYPose();
        return Math.sqrt(dx * dx + dy * dy);
    }


    // ═══════════════════════════════════════════
    //  TELEMETRY
    // ═══════════════════════════════════════════

    private void DisplayStatus() {
        telemetry.setAutoClear(true);

        switch (currentPhase) {
            case SelectingAlliance:
                telemetry.addLine("==========AUTONOMOUS CONSTRUCTOR==========");
                telemetry.addLine("");
                telemetry.addLine("  Select Alliance:");
                telemetry.addLine("    [X]  BLUE");
                telemetry.addLine("    [B]  RED");
                break;

            case SelectingStartingZone:
                telemetry.addLine("==========AUTONOMOUS CONSTRUCTOR==========");
                telemetry.addLine("");
                telemetry.addData("  Alliance", rConstants.Enums.selectedAlliance.name());
                telemetry.addLine("");
                telemetry.addLine("  Select Starting Zone:");
                telemetry.addLine("    [DPAD DOWN]  Close Zone");
                telemetry.addLine("    [DPAD UP]    Far Zone");
                break;

            case ConstructingActions:
                DisplayActionQueue();
                telemetry.addLine("");
                telemetry.addLine("-- ADD ACTIONS -----------------");
                telemetry.addLine("  [A]          Shoot Close Zone");
                telemetry.addLine("  [Y]          Shoot Far Zone");
                telemetry.addLine("  [DPAD UP]    Gate Collect");
                telemetry.addLine("  [DPAD LEFT]  Collect Close Set");
                telemetry.addLine("  [DPAD DOWN]  Collect Middle Set");
                telemetry.addLine("  [DPAD RIGHT] Collect Far Set");
                telemetry.addLine("");
                telemetry.addLine("  [BACK]   Undo Last Action");
                telemetry.addLine("  [START]  Confirm & Lock In All Actions");
                break;

            case Confirming:
                DisplayActionQueue();
                telemetry.addLine("");
                telemetry.addLine("== READY TO LOCK IN? ==========");
                telemetry.addLine("  [START] CONFIRM");
                telemetry.addLine("  [BACK]  Go Back & Edit");
                break;

            case Ready:
                DisplayActionQueue();
                telemetry.addLine("");
                telemetry.addLine("== AUTONOMOUS LOCKED IN ===================");
                telemetry.addLine("  Press PLAY to start");
                break;
        }

        telemetry.update();
    }

    private void DisplayActionQueue() {
        telemetry.addLine("==========AUTONOMOUS CONSTRUCTOR=========");
        telemetry.addLine("");
        telemetry.addData("  Alliance", rConstants.Enums.selectedAlliance.name());
        telemetry.addData("  Start Zone", isCloseZone ? "Close Zone" : "Far Zone");
        telemetry.addLine("");
        telemetry.addLine("-- ACTION QUEUE ----------------");

        int shootCount = 0;
        int ballCount = 0;

        for (int i = 0; i < actionQueue.size(); i++) {
            ActionType action = actionQueue.get(i);

            if (action == ActionType.ShootCloseZone || action == ActionType.ShootFarZone) {
                shootCount++;
                ballCount += 3;
                String zone = action == ActionType.ShootCloseZone ? "Close" : "Far";
                telemetry.addLine("  " + (i + 1) + ". Shoot #" + shootCount + " (" + zone + ")  [+3 | " + ballCount + " total]");
            } else {
                telemetry.addLine("  " + (i + 1) + ". " + getActionName(action));
            }
        }

        if (actionQueue.size() == 1) {
            telemetry.addLine("  ...");
            telemetry.addLine("  (add more actions below)");
        }

        telemetry.addLine("--------------------------------");
        telemetry.addData("  Total Balls", ballCount);
    }

    private String getActionName(ActionType action) {
        switch (action) {
            case ShootCloseZone:    return "Shoot (Close)";
            case ShootFarZone:      return "Shoot (Far)";
            case GateCollect:       return "Gate Collect";
            case CollectCloseSet:   return "Collect Close Set";
            case CollectMiddleSet:  return "Collect Middle Set";
            case CollectFarSet:     return "Collect Far Set";
            default:                return action.name();
        }
    }
}