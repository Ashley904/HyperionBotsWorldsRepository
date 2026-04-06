package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.AutonomousConstants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Configurable Autonomous", group = "Autonomous")
public class ConfigurableAutonomous extends OpMode {

    // ── Enums ──
    private enum Alliance { BLUE, RED }
    private enum Zone { CLOSE, FAR }
    private enum Phase { SELECT_ALLIANCE, SELECT_ZONE, BUILD_ACTIONS, CONFIRM, READY }
    private enum ActionType {
        SHOOT, GATE_COLLECT, COLLECT_CLOSE_SET,
        COLLECT_MIDDLE_SET, COLLECT_FAR_SET
    }

    // ── State ──
    private Phase currentPhase = Phase.SELECT_ALLIANCE;
    private Alliance selectedAlliance = null;
    private Zone selectedZone = null;
    private List<ActionType> actionQueue = new ArrayList<>();

    // ── Hardware ──
    private Follower follower;

    // ── Constants ──
    private AutonomousConstants.BlueCloseZone15SoloAutoConstants p;

    // ── Debounce ──
    private boolean prevX, prevCircle, prevTriangle;
    private boolean prevDpadLeft, prevDpadUp, prevDpadRight;
    private boolean prevBack, prevStart;


    // ═══════════════════════════════════════════
    //  FOLLOW PATH COMMAND
    // ═══════════════════════════════════════════

    private class FollowPathCommand extends CommandBase {
        private final PathChain path;
        private boolean started = false;

        public FollowPathCommand(PathChain path) {
            this.path = path;
        }

        @Override
        public void initialize() {
            follower.followPath(path);
            started = true;
        }

        @Override
        public void execute() {
            follower.update();
        }

        @Override
        public boolean isFinished() {
            return started && !follower.isBusy();
        }
    }


    // ═══════════════════════════════════════════
    //  OP MODE METHODS
    // ═══════════════════════════════════════════

    @Override
    public void init() {
        p = new AutonomousConstants.BlueCloseZone15SoloAutoConstants();
        CommandScheduler.getInstance().reset();
        // TODO: Initialize your follower
        // follower = new Follower(hardwareMap);
    }

    @Override
    public void init_loop() {
        switch (currentPhase) {
            case SELECT_ALLIANCE:
                handleAllianceSelection();
                break;
            case SELECT_ZONE:
                handleZoneSelection();
                break;
            case BUILD_ACTIONS:
                handleActionBuilding();
                break;
            case CONFIRM:
                handleConfirmation();
                break;
            case READY:
                break;
        }
        displayStatus();
        updateDebounce();
    }

    @Override
    public void start() {
        Pose startPose = getAlliancePose(p.startingPose);
        follower.setStartingPose(startPose);

        SequentialCommandGroup autoSequence = buildAutoSequence();
        CommandScheduler.getInstance().schedule(autoSequence);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }


    // ═══════════════════════════════════════════
    //  INIT LOOP HANDLERS
    // ═══════════════════════════════════════════

    private void handleAllianceSelection() {
        if (risingEdge(gamepad1.x, prevX)) {
            selectedAlliance = Alliance.BLUE;
            currentPhase = Phase.SELECT_ZONE;
        }
        if (risingEdge(gamepad1.b, prevCircle)) {
            selectedAlliance = Alliance.RED;
            currentPhase = Phase.SELECT_ZONE;
        }
    }

    private void handleZoneSelection() {
        if (risingEdge(gamepad1.dpad_left, prevDpadLeft)) {
            selectedZone = Zone.CLOSE;
            actionQueue.add(ActionType.SHOOT);
            currentPhase = Phase.BUILD_ACTIONS;
        }
        if (risingEdge(gamepad1.dpad_right, prevDpadRight)) {
            selectedZone = Zone.FAR;
            actionQueue.add(ActionType.SHOOT);
            currentPhase = Phase.BUILD_ACTIONS;
        }
    }

    private void handleActionBuilding() {
        if (risingEdge(gamepad1.x, prevX))
            actionQueue.add(ActionType.SHOOT);
        if (risingEdge(gamepad1.triangle, prevTriangle))
            actionQueue.add(ActionType.GATE_COLLECT);
        if (risingEdge(gamepad1.dpad_left, prevDpadLeft))
            actionQueue.add(ActionType.COLLECT_CLOSE_SET);
        if (risingEdge(gamepad1.dpad_up, prevDpadUp))
            actionQueue.add(ActionType.COLLECT_MIDDLE_SET);
        if (risingEdge(gamepad1.dpad_right, prevDpadRight))
            actionQueue.add(ActionType.COLLECT_FAR_SET);

        if (risingEdge(gamepad1.back, prevBack)) {
            if (actionQueue.size() > 1)
                actionQueue.remove(actionQueue.size() - 1);
        }
        if (risingEdge(gamepad1.start, prevStart))
            currentPhase = Phase.CONFIRM;
    }

    private void handleConfirmation() {
        if (risingEdge(gamepad1.start, prevStart))
            currentPhase = Phase.READY;
        if (risingEdge(gamepad1.back, prevBack))
            currentPhase = Phase.BUILD_ACTIONS;
    }


    // ═══════════════════════════════════════════
    //  BALL COUNTER
    // ═══════════════════════════════════════════

    private int calculateTotalBalls() {
        int count = 0;
        for (ActionType action : actionQueue) {
            if (action == ActionType.SHOOT) {
                count += 3;
            }
        }
        return count;
    }

    private int calculateRunningBalls(int upToIndex) {
        int count = 0;
        for (int i = 0; i <= upToIndex; i++) {
            if (actionQueue.get(i) == ActionType.SHOOT) {
                count += 3;
            }
        }
        return count;
    }

    private int calculateShootNumber(int upToIndex) {
        int count = 0;
        for (int i = 0; i <= upToIndex; i++) {
            if (actionQueue.get(i) == ActionType.SHOOT) {
                count++;
            }
        }
        return count;
    }


    // ═══════════════════════════════════════════
    //  BUILD COMMAND SEQUENCE
    // ═══════════════════════════════════════════

    private SequentialCommandGroup buildAutoSequence() {
        List<CommandBase> commands = new ArrayList<>();
        Pose trackingPose = getAlliancePose(p.startingPose);

        for (ActionType action : actionQueue) {
            switch (action) {
                case SHOOT:
                    commands.add(buildShootCommands(trackingPose));
                    trackingPose = getAlliancePose(p.scorePreloadPose);
                    break;
                case GATE_COLLECT:
                    commands.add(buildGateCollectCommands(trackingPose));
                    trackingPose = getAlliancePose(p.gateCollect1);
                    break;
                case COLLECT_CLOSE_SET:
                    commands.add(buildCollectCloseSetCommands(trackingPose));
                    trackingPose = getAlliancePose(p.collectCloseSetPose2);
                    break;
                case COLLECT_MIDDLE_SET:
                    commands.add(buildCollectMiddleSetCommands(trackingPose));
                    trackingPose = getAlliancePose(p.collectMiddleSetPose2);
                    break;
                case COLLECT_FAR_SET:
                    commands.add(buildCollectFarSetCommands(trackingPose));
                    trackingPose = getAlliancePose(p.collectFarSetPose2);
                    break;
            }
        }

        return new SequentialCommandGroup(commands.toArray(new CommandBase[0]));
    }


    // ═══════════════════════════════════════════
    //  ACTION COMMAND BUILDERS
    // ═══════════════════════════════════════════

    private SequentialCommandGroup buildShootCommands(Pose fromPose) {
        Pose scorePose = getAlliancePose(p.scorePreloadPose);

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(fromPose.getX(), fromPose.getY()),
                        new Pose(scorePose.getX(), scorePose.getY())))
                .setLinearHeadingInterpolation(fromPose.getHeading(), scorePose.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(path)
                // TODO: Add shoot command
        );
    }

    private SequentialCommandGroup buildGateCollectCommands(Pose fromPose) {
        Pose gatePose = getAlliancePose(p.gateCollect1);

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(fromPose.getX(), fromPose.getY()),
                        new Pose(gatePose.getX(), gatePose.getY())))
                .setLinearHeadingInterpolation(fromPose.getHeading(), gatePose.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(path)
                // TODO: Add intake command
        );
    }

    private SequentialCommandGroup buildCollectCloseSetCommands(Pose fromPose) {
        Pose pose1 = getAlliancePose(p.collectCloseSetPose1);
        Pose pose2 = getAlliancePose(p.collectCloseSetPose2);

        PathChain driveTo = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(fromPose.getX(), fromPose.getY()),
                        new Pose(pose1.getX(), pose1.getY())))
                .setLinearHeadingInterpolation(fromPose.getHeading(), pose1.getHeading())
                .build();

        PathChain collect = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pose1.getX(), pose1.getY()),
                        new Pose(pose2.getX(), pose2.getY())))
                .setConstantHeadingInterpolation(pose2.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(driveTo),
                new FollowPathCommand(collect)
                // TODO: Add intake command
        );
    }

    private SequentialCommandGroup buildCollectMiddleSetCommands(Pose fromPose) {
        Pose pose1 = getAlliancePose(p.collectMiddleSetPose1);
        Pose pose2 = getAlliancePose(p.collectMiddleSetPose2);

        PathChain driveTo = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(fromPose.getX(), fromPose.getY()),
                        new Pose(pose1.getX(), pose1.getY())))
                .setLinearHeadingInterpolation(fromPose.getHeading(), pose1.getHeading())
                .build();

        PathChain collect = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pose1.getX(), pose1.getY()),
                        new Pose(pose2.getX(), pose2.getY())))
                .setConstantHeadingInterpolation(pose2.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(driveTo),
                new FollowPathCommand(collect)
                // TODO: Add intake command
        );
    }

    private SequentialCommandGroup buildCollectFarSetCommands(Pose fromPose) {
        Pose pose1 = getAlliancePose(p.collectFareSetPose1);
        Pose pose2 = getAlliancePose(p.collectFarSetPose2);

        PathChain driveTo = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(fromPose.getX(), fromPose.getY()),
                        new Pose(pose1.getX(), pose1.getY())))
                .setLinearHeadingInterpolation(fromPose.getHeading(), pose1.getHeading())
                .build();

        PathChain collect = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pose1.getX(), pose1.getY()),
                        new Pose(pose2.getX(), pose2.getY())))
                .setConstantHeadingInterpolation(pose2.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(driveTo),
                new FollowPathCommand(collect)
                // TODO: Add intake command
        );
    }


    // ═══════════════════════════════════════════
    //  TELEMETRY
    // ═══════════════════════════════════════════

    private void displayStatus() {
        telemetry.setAutoClear(true);

        switch (currentPhase) {
            case SELECT_ALLIANCE:
                telemetry.addLine("╔══════════════════════════╗");
                telemetry.addLine("║   AUTONOMOUS  BUILDER    ║");
                telemetry.addLine("╚══════════════════════════╝");
                telemetry.addLine("");
                telemetry.addLine("  Select Alliance:");
                telemetry.addLine("    [SQUARE]  BLUE");
                telemetry.addLine("    [CIRCLE]  RED");
                break;

            case SELECT_ZONE:
                telemetry.addLine("╔══════════════════════════╗");
                telemetry.addLine("║   AUTONOMOUS  BUILDER    ║");
                telemetry.addLine("╚══════════════════════════╝");
                telemetry.addLine("");
                telemetry.addData("  Alliance", selectedAlliance.name());
                telemetry.addLine("");
                telemetry.addLine("  Select Starting Zone:");
                telemetry.addLine("    [DPAD LEFT]  Close Zone");
                telemetry.addLine("    [DPAD RIGHT] Far Zone");
                break;

            case BUILD_ACTIONS:
                displayActionQueue();
                telemetry.addLine("");
                telemetry.addLine("-- ADD ACTIONS -----------------");
                telemetry.addLine("  [SQUARE]     Shoot");
                telemetry.addLine("  [TRIANGLE]   Gate Collect");
                telemetry.addLine("  [DPAD LEFT]  Collect Close Set");
                telemetry.addLine("  [DPAD UP]    Collect Middle Set");
                telemetry.addLine("  [DPAD RIGHT] Collect Far Set");
                telemetry.addLine("");
                telemetry.addLine("  [SHARE]   Undo Last");
                telemetry.addLine("  [OPTIONS] Confirm & Lock In");
                break;

            case CONFIRM:
                displayActionQueue();
                telemetry.addLine("");
                telemetry.addLine("== READY TO LOCK IN? ==========");
                telemetry.addLine("  [OPTIONS] CONFIRM");
                telemetry.addLine("  [SHARE]   Go Back & Edit");
                break;

            case READY:
                displayActionQueue();
                telemetry.addLine("");
                telemetry.addLine("== LOCKED IN ===================");
                telemetry.addLine("  Press PLAY to start");
                break;
        }

        telemetry.update();
    }

    private void displayActionQueue() {
        int totalBalls = calculateTotalBalls();

        telemetry.addLine("╔══════════════════════════╗");
        telemetry.addLine("║   AUTONOMOUS  BUILDER    ║");
        telemetry.addLine("╚══════════════════════════╝");
        telemetry.addLine("");
        telemetry.addData("  Alliance", selectedAlliance.name());
        telemetry.addData("  Start Zone", selectedZone.name());
        telemetry.addLine("");
        telemetry.addLine("-- ACTION QUEUE ----------------");

        for (int i = 0; i < actionQueue.size(); i++) {
            ActionType action = actionQueue.get(i);

            if (action == ActionType.SHOOT) {
                int shootNum = calculateShootNumber(i);
                int ballsSoFar = calculateRunningBalls(i);
                telemetry.addLine("  " + (i + 1) + ". Shoot #" + shootNum + "  [+3 | " + ballsSoFar + " total]");
                telemetry.addLine("       -> Drive to Score Position");
                telemetry.addLine("       -> Shoot 3 Balls");
            } else {
                telemetry.addLine("  " + (i + 1) + ". " + getActionName(action));

                String[] steps = getActionSteps(action);
                for (String step : steps) {
                    telemetry.addLine("       " + step);
                }
            }
        }

        if (actionQueue.size() == 1) {
            telemetry.addLine("  ...");
            telemetry.addLine("  (add more actions below)");
        }

        telemetry.addLine("--------------------------------");
        telemetry.addData("  Total Actions", actionQueue.size());
        telemetry.addData("  Total Balls", totalBalls + " balls");
    }

    private String getActionName(ActionType action) {
        switch (action) {
            case SHOOT:              return "Shoot";
            case GATE_COLLECT:       return "Gate Collect";
            case COLLECT_CLOSE_SET:  return "Collect Close Set";
            case COLLECT_MIDDLE_SET: return "Collect Middle Set";
            case COLLECT_FAR_SET:    return "Collect Far Set";
            default:                 return action.name();
        }
    }

    private String[] getActionSteps(ActionType action) {
        switch (action) {
            case GATE_COLLECT:
                return new String[]{
                        "-> Drive to Gate",
                        "-> Intake from Gate"
                };
            case COLLECT_CLOSE_SET:
                return new String[]{
                        "-> Drive to Close Set",
                        "-> Drive Forward + Intake"
                };
            case COLLECT_MIDDLE_SET:
                return new String[]{
                        "-> Drive to Middle Set",
                        "-> Drive Forward + Intake"
                };
            case COLLECT_FAR_SET:
                return new String[]{
                        "-> Drive to Far Set",
                        "-> Drive Forward + Intake"
                };
            default:
                return new String[]{};
        }
    }


    // ═══════════════════════════════════════════
    //  UTILITY
    // ═══════════════════════════════════════════

    private Pose getAlliancePose(Pose bluePose) {
        return selectedAlliance == Alliance.RED ? bluePose.mirror() : bluePose;
    }

    private boolean risingEdge(boolean current, boolean previous) {
        return current && !previous;
    }

    private void updateDebounce() {
        prevX = gamepad1.x;
        prevCircle = gamepad1.b;
        prevTriangle = gamepad1.triangle;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadRight = gamepad1.dpad_right;
        prevBack = gamepad1.back;
        prevStart = gamepad1.start;
    }
}