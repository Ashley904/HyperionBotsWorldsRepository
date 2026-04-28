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
    public static double closeZoneFlyWheelVelocity = 1130.0;
    public static double closeZoneHoodPosition = 0.69;

    public static double farZoneFlyWheelVelocity = 1460.0;
    public static double farZoneHoodPosition = 0.52;





    private enum Phase { SelectingAlliance, SelectingStartingZone, ConstructingActions, Confirming, Ready }
    private enum ActionType { ShootCloseZone, ShootFarZone, CollectCloseSet, CollectMiddleSet, CollectFarSet, GateCollect, OpenGate, HumanPlayerZoneCollect }

    public static double collectSlowSpeed = 0.75;


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
    private ButtonReader openGateButtonReader, humanPlayerZoneCollectButtonReader;
    private ButtonReader undoButtonReader, confirmButtonReader;


    int currentSpindexerIndex = 0;


    //----------Precomputed Shooter Lookup Table----------//
    private static final double LOOKUP_TABLE_RESOLUTION = 0.01;
    private static final double LOOKUP_TABLE_MAX_DISTANCE = 200.0;
    private static final int LOOKUP_TABLE_SIZE = (int) (LOOKUP_TABLE_MAX_DISTANCE / LOOKUP_TABLE_RESOLUTION) + 1;
    private final double[] precomputedFlyWheelVelocities = new double[LOOKUP_TABLE_SIZE];
    private final double[] precomputedHoodPositions = new double[LOOKUP_TABLE_SIZE];
    //----------end----------//


    // ═══════════════════════════════════════════
    //  FOLLOW PATH COMMAND
    // ═══════════════════════════════════════════

    private class FollowPathCommand extends CommandBase {
        private final Pose toPose;
        private final double maxSpeed;

        public FollowPathCommand(Pose toPose) {
            this.toPose = toPose;
            this.maxSpeed = 1.0;
        }

        public FollowPathCommand(Pose toPose, double maxSpeed) {
            this.toPose = toPose;
            this.maxSpeed = maxSpeed;
        }

        @Override
        public void initialize() {
            Pose fromPose = follower.getPose();
            double startHeading = fromPose.getHeading();
            double endHeading = toPose.getHeading();

            // Normalize so heading takes the shortest rotation path
            double delta = endHeading - startHeading;
            while (delta > Math.PI)  delta -= 2 * Math.PI;
            while (delta < -Math.PI) delta += 2 * Math.PI;
            double adjustedEndHeading = startHeading + delta;

            PathChain pathChain = follower.pathBuilder()
                    .addPath(new BezierLine(fromPose, toPose))
                    .setLinearHeadingInterpolation(startHeading, adjustedEndHeading)
                    .build();
            follower.followPath(pathChain, maxSpeed, true);   // hold position = true
        }

        @Override public void execute() {}
        @Override public boolean isFinished() { return !follower.isBusy(); }
        @Override public void end(boolean interrupted) {}
    }


    // ═══════════════════════════════════════════
    //  OP MODE METHODS
    // ════════════════════════════════════════





    private int loopCount = 0;

    @Override
    public void init() {
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);

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
            startPose = isBlue
                    ? rConstants.AutonomousPositionConstants.blueCloseZoneStartingPosition
                    : rConstants.AutonomousPositionConstants.redCloseZoneStartingPosition;
        } else {
            startPose = isBlue
                    ? rConstants.AutonomousPositionConstants.blueFarZoneStartingPosition
                    : rConstants.AutonomousPositionConstants.redFarZoneStartingPosition;
        }
        follower.setStartingPose(startPose);
        trackingPose = startPose;

        shooterSubsystem.setTargetVelocity(1300);

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
        turretSubsystem.setTurretAngle(0);

        // Auto-indexing runs every loop while intake is on
        SpindexerManaging();
        CommandScheduler.getInstance().run();

        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Distance To Goal", "%.1f", getDistanceToGoal());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Intake State", intakeSubsystem.getState());
        telemetry.addData("Spindexer State", spindexerSubsystem.getSpindexerState());
        telemetry.update();
    }






    @Override
    public void stop(){
        rConstants.FieldConstants.autonomousEndPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
    }






    private void SpindexerManaging(){
        if(spindexerSubsystem.getSpindexerState() == SpindexerSubsystem.SpindexerState.Shooting
                || intakeSubsystem.getState() != IntakeSubsystem.IntakeState.Intaking
                || !spindexerSubsystem.spindexerPositionReached()) return;

        double leftDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
        boolean ballPresent = leftDistance <= rConstants.SensorConstants.distanceSensorOccupiedThreshold;

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
        openGateButtonReader            = new ButtonReader(gamepad1EX, GamepadKeys.Button.LEFT_BUMPER);
        humanPlayerZoneCollectButtonReader = new ButtonReader(gamepad1EX, GamepadKeys.Button.RIGHT_BUMPER);
        undoButtonReader                = new ButtonReader(gamepad1EX, GamepadKeys.Button.BACK);
        confirmButtonReader             = new ButtonReader(gamepad1EX, GamepadKeys.Button.START);
    }

    private void ReadAllButtons() {
        selectBlueAllianceButtonReader.readValue();  selectRedAllianceButtonReader.readValue();
        selectCloseZoneButtonReader.readValue();     selectFarZoneButtonReader.readValue();
        shootCloseZoneButtonReader.readValue();      shootFarZoneButtonReader.readValue();
        collectCloseSetButtonReader.readValue();     collectMiddleSetButtonReader.readValue();
        collectFarSetButtonReader.readValue();       gateCollectButtonReader.readValue();
        openGateButtonReader.readValue();            humanPlayerZoneCollectButtonReader.readValue();
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
        if (openGateButtonReader.wasJustPressed()) actionQueue.add(ActionType.OpenGate);
        if (humanPlayerZoneCollectButtonReader.wasJustPressed()) actionQueue.add(ActionType.HumanPlayerZoneCollect);

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
                    Pose target = isBlue
                            ? rConstants.AutonomousPositionConstants.scoreCloseZoneBlueSide
                            : rConstants.AutonomousPositionConstants.scoreCloseZoneRedSide;
                    commands.add(new FollowPathCommand(target));
                    commands.add(new InstantCommand(() -> {
                        shooterSubsystem.setTargetVelocity(closeZoneFlyWheelVelocity);
                        shooterSubsystem.setHoodPosition(closeZoneHoodPosition);
                    }));
                    commands.add(new WaitUntilCommand(() ->
                            rConstants.Enums.currentShooterState == rConstants.Enums.ShooterState.TargetReached));
                    commands.add(new ShootArtefactsCMD(spindexerSubsystem, intakeSubsystem, robot));
                    commands.add(new WaitUntilCommand(() ->
                            spindexerSubsystem.getSpindexerState() != SpindexerSubsystem.SpindexerState.Shooting));
                    break;
                }

                case ShootFarZone: {
                    Pose target = isBlue
                            ? rConstants.AutonomousPositionConstants.scoreFarZoneBlueSide
                            : rConstants.AutonomousPositionConstants.scoreFarZoneRedSide;
                    commands.add(new FollowPathCommand(target));
                    commands.add(new InstantCommand(() -> {
                        shooterSubsystem.setTargetVelocity(farZoneFlyWheelVelocity);
                        shooterSubsystem.setHoodPosition(farZoneHoodPosition);
                    }));
                    commands.add(new WaitUntilCommand(() ->
                            rConstants.Enums.currentShooterState == rConstants.Enums.ShooterState.TargetReached));
                    commands.add(new ShootArtefactsCMD(spindexerSubsystem, intakeSubsystem, robot));
                    commands.add(new WaitUntilCommand(() ->
                            spindexerSubsystem.getSpindexerState() != SpindexerSubsystem.SpindexerState.Shooting));
                    break;
                }

                case GateCollect: {
                    Pose target = isBlue
                            ? rConstants.AutonomousPositionConstants.gateCollectBlueSide
                            : rConstants.AutonomousPositionConstants.gateCollectRedSide;
                    commands.add(new FollowPathCommand(target));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new WaitCommand(1300));
                    break;
                }

                case CollectCloseSet: {
                    Pose pose1 = isBlue
                            ? rConstants.AutonomousPositionConstants.collectThirdSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectThirdSet1RedSide;
                    Pose pose2 = isBlue
                            ? rConstants.AutonomousPositionConstants.collectThirdSet2BlueSide
                            : rConstants.AutonomousPositionConstants.collectThirdSet2RedSide;
                    commands.add(new FollowPathCommand(pose1));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new FollowPathCommand(pose2, collectSlowSpeed));
                    break;
                }

                case CollectMiddleSet: {
                    Pose pose1 = isBlue
                            ? rConstants.AutonomousPositionConstants.collectSecondSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectSecondSet1RedSide;
                    Pose pose2 = isBlue
                            ? rConstants.AutonomousPositionConstants.collectSecondSet2BlueSide
                            : rConstants.AutonomousPositionConstants.collectSecondSet2RedSide;

                    commands.add(new FollowPathCommand(pose1));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new FollowPathCommand(pose2, collectSlowSpeed));

                    commands.add(new CommandBase() {
                        private Pose dodgePose;

                        @Override
                        public void initialize() {
                            Pose current = follower.getPose();
                            dodgePose = new Pose(
                                    current.getX() + 2.5,
                                    current.getY() - 5,
                                    current.getHeading()
                            );
                            PathChain pathChain = follower.pathBuilder()
                                    .addPath(new BezierLine(current, dodgePose))
                                    .setLinearHeadingInterpolation(current.getHeading(), dodgePose.getHeading())
                                    .build();
                            follower.followPath(pathChain, 1.0, true);
                        }

                        @Override public void execute() {}
                        @Override public boolean isFinished() { return !follower.isBusy(); }
                        @Override public void end(boolean interrupted) {}
                    });

                    break;
                }

                case CollectFarSet: {
                    Pose pose1 = isBlue
                            ? rConstants.AutonomousPositionConstants.collectFirstSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectFirstSet1RedSide;
                    Pose pose2 = isBlue
                            ? rConstants.AutonomousPositionConstants.collectFirstSet2BlueSide
                            : rConstants.AutonomousPositionConstants.collectFirstSet2RedSide;
                    commands.add(new FollowPathCommand(pose1));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    commands.add(new FollowPathCommand(pose2, collectSlowSpeed ));
                    break;
                }

                case OpenGate: {
                    Pose pose1 = isBlue
                            ? rConstants.AutonomousPositionConstants.openGateBlueSidePose1
                            : rConstants.AutonomousPositionConstants.openGateRedSidePose1;
                    Pose pose2 = isBlue
                            ? rConstants.AutonomousPositionConstants.openGateBlueSidePose2
                            : rConstants.AutonomousPositionConstants.openGateRedSidePose2;
                    commands.add(new FollowPathCommand(pose1));
                    commands.add(new FollowPathCommand(pose2));
                    break;
                }

                case HumanPlayerZoneCollect: {
                    Pose pose1 = isBlue
                            ? rConstants.AutonomousPositionConstants.humanPlayerZoneBlueSideCollectPose1
                            : rConstants.AutonomousPositionConstants.humanPlayerZoneRedSideCollectPose1;
                    Pose pose2 = isBlue
                            ? rConstants.AutonomousPositionConstants.humanPlayerZoneBlueSideCollectPose2
                            : rConstants.AutonomousPositionConstants.humanPlayerZoneRedSideCollectPose2;
                    commands.add(new FollowPathCommand(pose1));
                    commands.add(new InstantCommand(() -> intakeSubsystem.setState(IntakeSubsystem.IntakeState.Intaking)));
                    new WaitCommand(350); //TODO: change thins to make the robot wait for longer before moving on
                    commands.add(new FollowPathCommand(pose2, 0.65));
                    break;
                }
            }
        }

        return new SequentialCommandGroup(commands.toArray(new CommandBase[0]));
    }







    // ═══════════════════════════════════════════
    //  HELPERS
    // ═══════════════════════════════════════════

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
        return follower.getPose().distanceFrom(getActiveGoalPose());
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
                telemetry.addLine("  [LB]         Open Gate");
                telemetry.addLine("  [RB]         Human Player Zone Collect");
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
            case ShootCloseZone:           return "Shoot (Close)";
            case ShootFarZone:             return "Shoot (Far)";
            case GateCollect:              return "Gate Collect";
            case CollectCloseSet:          return "Collect Close Set";
            case CollectMiddleSet:         return "Collect Middle Set";
            case CollectFarSet:            return "Collect Far Set";
            case OpenGate:                 return "Open Gate";
            case HumanPlayerZoneCollect:   return "Human Player Zone Collect";
            default:                       return action.name();
        }
    }
}