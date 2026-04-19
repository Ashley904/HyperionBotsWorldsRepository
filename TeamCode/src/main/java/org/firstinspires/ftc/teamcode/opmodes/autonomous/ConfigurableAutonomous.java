package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.customPathing.Drive;
import org.firstinspires.ftc.teamcode.customPathing.Point;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Worlds Configurable Autonomous", group = "Autonomous")
public class ConfigurableAutonomous extends OpMode {

    // ── Enums ──
    private enum Phase { SelectingAlliance, SelectingStartingZone, ConstructingActions, Confirming, Ready }
    private enum ActionType { ShootCloseZone, ShootFarZone, CollectCloseSet, CollectMiddleSet, CollectFarSet, GateCollect }





    // ── Hardware ──
    private RobotHardwareMap robot;
    private Drive drive;





    // ── Live Pose ──
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentHeading = 0.0;





    // ── State ──
    private Phase currentPhase = Phase.SelectingAlliance;
    private boolean isCloseZone = true;
    private List<ActionType> actionQueue = new ArrayList<>();





    // ── Button Readers ──
    private ButtonReader selectBlueAllianceButtonReader, selectRedAllianceButtonReader;
    private ButtonReader selectCloseZoneButtonReader, selectFarZoneButtonReader;
    private ButtonReader shootCloseZoneButtonReader, shootFarZoneButtonReader;
    private ButtonReader collectCloseSetButtonReader, collectMiddleSetButtonReader, collectFarSetButtonReader;
    private ButtonReader gateCollectButtonReader;
    private ButtonReader undoButtonReader, confirmButtonReader;





    // Drive To Point Command
    private class DriveToPointCommand extends CommandBase {
        private final Point targetPoint;

        public DriveToPointCommand(Point targetPoint) { this.targetPoint = targetPoint; }

        @Override public void initialize() { drive.setTargetPoint(targetPoint); drive.trajectoryStartSequence(); }
        @Override public void execute() { drive.driveToTargetPoint(currentHeading, currentX, currentY); }
        @Override public boolean isFinished() { return drive.isAtTarget(currentX, currentY); }
        @Override public void end(boolean interrupted) { drive.stopMotor(); }
    }







    @Override
    public void init() {
        robot = new RobotHardwareMap();
        robot.init(hardwareMap);
        robot.pinpointDriver.recalibrateIMU();
        robot.pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        drive = new Drive(robot);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();
        InitializeButtonReaders();

        telemetry.addData("Status", "Initialization Complete... Ready to Start!");
        telemetry.update();
    }





    @Override
    public void init_loop() {
        ReadAllButtons();

        switch (currentPhase) {
            case SelectingAlliance: HandleAllianceSelection(); break;
            case SelectingStartingZone: HandleZoneSelection(); break;
            case ConstructingActions: HandleActionBuilding(); break;
            case Confirming: HandleConfirmation(); break;
            case Ready: break;
        }

        DisplayStatus();
    }





    @Override
    public void start() { CommandScheduler.getInstance().schedule(BuildAutoSequence()); }





    @Override
    public void loop() {
        robot.pinpointDriver.update();
        currentX = rConstants.FieldConstants.startingXPosition + robot.pinpointDriver.getPosX(DistanceUnit.INCH);
        currentY = rConstants.FieldConstants.startingYPosition + robot.pinpointDriver.getPosY(DistanceUnit.INCH);
        currentHeading = Math.toDegrees(robot.pinpointDriver.getHeading(AngleUnit.RADIANS));

        CommandScheduler.getInstance().run();

        telemetry.addData("Current X Position: ", "%.1f", currentX);
        telemetry.addData("Current Y Position: ", "%.1f", currentY);
        telemetry.addData("Heading", "%.2f", Math.toDegrees(currentHeading));
        telemetry.update();
    }





    @Override
    public void stop() { drive.stopMotor(); CommandScheduler.getInstance().reset(); }





    // ═══════════════════════════════════════════
    //  BUTTON READERS
    // ═══════════════════════════════════════════

    private void InitializeButtonReaders() {
        GamepadEx gamepad1EX = new GamepadEx(gamepad1);

        selectBlueAllianceButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectBlueAlliance);
        selectRedAllianceButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectRedAlliance);
        selectCloseZoneButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectCloseZoneStartingPosition);
        selectFarZoneButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.selectFarZoneStartingPosition);
        shootCloseZoneButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.shootCloseZoneMapping);
        shootFarZoneButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.shootFarZoneMapping);
        collectCloseSetButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.collectCloseSetMapping);
        collectMiddleSetButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.collectMiddleSetMapping);
        collectFarSetButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.collectFarSetMapping);
        gateCollectButtonReader = new ButtonReader(gamepad1EX, rConstants.GamePadControls.gateCollectMapping);
        undoButtonReader = new ButtonReader(gamepad1EX, GamepadKeys.Button.BACK);
        confirmButtonReader = new ButtonReader(gamepad1EX, GamepadKeys.Button.START);
    }

    private void ReadAllButtons() {
        selectBlueAllianceButtonReader.readValue();  selectRedAllianceButtonReader.readValue();
        selectCloseZoneButtonReader.readValue();     selectFarZoneButtonReader.readValue();
        shootCloseZoneButtonReader.readValue();      shootFarZoneButtonReader.readValue();
        collectCloseSetButtonReader.readValue();     collectMiddleSetButtonReader.readValue();
        collectFarSetButtonReader.readValue();       gateCollectButtonReader.readValue();
        undoButtonReader.readValue();                confirmButtonReader.readValue();
    }






    private void HandleAllianceSelection() { // Selecting Alliance Color
        if (selectBlueAllianceButtonReader.wasJustPressed()) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.BLUE; currentPhase = Phase.SelectingAlliance; }
        if (selectRedAllianceButtonReader.wasJustPressed()) { rConstants.Enums.selectedAlliance = rConstants.Enums.Alliance.RED; currentPhase = Phase.SelectingAlliance; }
    }

    private void HandleZoneSelection() { // Selecting Starting Zone
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






    // Building Command Sequence
    private SequentialCommandGroup BuildAutoSequence() {
        List<CommandBase> commands = new ArrayList<>();
        boolean isBlue = rConstants.Enums.selectedAlliance == rConstants.Enums.Alliance.BLUE;

        for (ActionType action : actionQueue) {
            switch (action) {
                case ShootCloseZone:
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.scoreCloseZoneBlueSide
                            : rConstants.AutonomousPositionConstants.scoreCloseZoneRedSide));
                    // TODO: add shoot commands
                    break;

                case ShootFarZone:
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.scoreFarZoneBlueSide
                            : rConstants.AutonomousPositionConstants.scoreFarZoneRedSide));
                    // TODO: add shoot command
                    break;

                case GateCollect:
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.gateCollectBlueSide
                            : rConstants.AutonomousPositionConstants.gateCollectRedSide));
                    // TODO: add intake command
                    break;

                case CollectCloseSet:
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.collectThirdSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectThirdSet2BlueSide));
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.collectThirdSet1RedSide
                            : rConstants.AutonomousPositionConstants.collectThirdSet2RedSide));
                    // TODO: add intake command
                    break;

                case CollectMiddleSet:
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.collectSecondSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectSecondSet2BlueSide));
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.collectSecondSet1RedSide
                            : rConstants.AutonomousPositionConstants.collectSecondSet2RedSide));
                    // TODO: add intake command
                    break;

                case CollectFarSet:
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.collectFirstSet1BlueSide
                            : rConstants.AutonomousPositionConstants.collectFirstSet2BlueSide));
                    commands.add(new DriveToPointCommand(isBlue
                            ? rConstants.AutonomousPositionConstants.collectFirstSet1RedSide
                            : rConstants.AutonomousPositionConstants.collectFirstSet2RedSide));
                    // TODO: add intake command
                    break;
            }
        }

        return new SequentialCommandGroup(commands.toArray(new CommandBase[0]));
    }





    // Telemetry Dispalying
    private void DisplayStatus() {
        telemetry.setAutoClear(true);

        switch (currentPhase) {
            case SelectingAlliance:
                telemetry.addLine("=========AUTONOMOUS CONSTRUCTOR==========");
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