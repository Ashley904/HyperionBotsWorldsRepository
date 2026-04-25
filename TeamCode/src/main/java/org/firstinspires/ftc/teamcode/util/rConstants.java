package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;

public class rConstants {
    @Config
    public static class FieldConstants{
        public static Pose blueGoalPose = new Pose(136.0, 133.0);
        public static Pose redGoalPose = new Pose(131.0, 9.0);





        public static Pose startingPose = new Pose(72.0, 72.0);
        public static Pose resetPose = new Pose(133.0, 72.0);
    }





    @Config
    public static class Enums{
        public enum Alliance {BLUE, RED}
        public static Alliance selectedAlliance = Alliance.RED;





        public enum DriveMode {RobotCentric, FieldCentric}
        public static DriveMode selectedDriveMode = DriveMode.FieldCentric;





        public enum ShooterState {Disabled, Accelerating, TargetReached}
        public static ShooterState currentShooterState = ShooterState.Disabled;





        public enum StartingPosition {BlueClose, BlueFar, RedClose, RedFar}
        public static StartingPosition selectedStartingPosition = StartingPosition.RedClose;
    }





    @Config
    public static class AllianceHeadingOffsets{
        public static double blueAllianceHeadingOffset = 90;
        public static double redAllianceHeadingOffset = -45;
    }





    @Config
    public static class DriveTrainConstants{
        public static String frontLeftMotorName = "FL";
        public static String backLeftMotorName = "BL";
        public static String frontRightMotorName = "FR";
        public static String backRightMotorName = "BR";





        public static Boolean frontLeftInverted = false;
        public static Boolean backLeftInverted = false;
        public static Boolean frontRightInverted = true;
        public static Boolean backRightInverted = true;





        public static Boolean floatModeEnabled = false;





        public static double headingKp=0.0, headingKd=0.0;
        public static double autoLockHeadingTolerance = 5.0;





        public static double driveCubicTerm=0.6, driveLinearTerm=0.5;
        public static double maximumDriveTrainSpeed=1.0, minimumDriveTrainSpeed=0.525;// Lower this value if you want ot be able ton drvie at a slower spped when holding right trigger
    }





    @Config
    public static class PinpointConstants{
        public static String pinpointDriveName = "pinpoint";





        public static boolean strafePodInverted = false;
        public static boolean forwardPodInverted = true;






        public static double strafePodOffset = 0.79;
        public static double forwardPodOffset = -7.68;
    }





    @Config
    public static class AutonomousConstants{
        public static double translationalKSQ = 100.0;
        public static double translationalKD = 0.0;

        public static double headingKSQ = 100.0;
        public static double headingKd = 0.0;




        public static double radiusTolerance = 15.0;
        public static double headingTolerance = 5.0; // Degrees






        public static double targetConfirmationSeconds = 0.25;
        public static double overtimeProtectionSeconds = 3;
    }





    @Config
    public static class IntakeConstants{
        public static String intakeMotorName = "intakeMotor";
        public static Boolean intakeInverted = false;
        public static Boolean floatModeEnabled = false;





        public static double disabledSpeed = 0.0;//to keep intake on if needed (.2 max)
        public static double intakingSpeed = 1.0;
        public static double reversingSpeed = -1.0;
        public static double idlingSpeed = 0.9;





        public static double currentAlert = 6.25;
        public static double jamVelocityAlert = 600.0;
        public static double jamTimeConfirmation = 100;
    }





    @Config
    public static class SpindexerConstants{
        public static String leftSpindexerServoName = "leftSpindexerServo";
        public static String rightSpindexerServoName = "rightSpindexerServo";
        public static String spindexerEncoderName = DriveTrainConstants.backLeftMotorName;





        public static double[] intakingPositions = {0.175, 0.415, 0.66};
        public static double[] shootingPositions = {0.285, 0.535, 0.78};
        public static double[] encoderShootingPositions = {1300, 4040, 6760};




        public static double spindexerVelocityCheckDelay = 10;
        public static double minimumPeakVelocity = 1000;
        public static double decelerationTriggerRatio = 0.25;




        public static double positionalTolerance = 1200;
        public static double positionalPercentageTolerance = 0.815;// The higher this value is the longer the transfer will wait before it tranfers
    }





    @Config
    public static class ShooterConstants{
        public static String leftFlyWheelMotorName = "leftFlyWheelMotor";
        public static String rightFlyWheelMotorName = "rightFlyWheelMotor";





        public static String hoodServoName = "hoodServo";






        public static boolean leftFlyWheelMotorInverted = true; //if flywheel goes backwards, change to false.
        public static boolean rightFlyWheelMotorInverted = false;
        public static boolean floatModeEnabled=false;



        public static double flyWheelKs=0.17, flyWheelKf=0.00036, flyWheelKp=0.002; // Retune these if needed
        public static double nominalVoltage=13.4;






        public static double maximumFlyWheelVelocity=2120;  // Retune these if needed
        public static double velocityReachedTolerance=65.0;





        public static double flywheelToBallSpeedRatio = 0.15;





        public static double minimumHoodPosition=0.25, maximumHoodPosition=0.68;  // Retune these if needed
    }





    @Config
    public static class TurretConstants{
        public static String turretMotorName = "turretMotor";
        public static boolean turretMotorInverted = false;
        public static boolean floatModeEnabled = false;





        public static double turretKp=0.025, turretKd=0.35;
        public static double maxTurretPower=0.75;





        public static double turretMinAngle = -50, turretMaxAngle = 50;
        public static double turretOffset = 0;
    }





    @Config
    public static class LimelightAutoAimConstants{
        public static double limelightKp = 0.03;
        public static double limelightKd = 0.00215;





        public static double maxCorrectionPower = 0.85;
        public static double limelightTurnDirection = -1;
        public static double limelightHeadingTolerance = 2;
        public static double targetingOffset = 5.0;  // Play around with this on the ftc dashboard if you want to offset your limelight targetting
        public static double targetingOffsetFarDistance = 135.0;
        public static double targetingOffsetFarScalar = 0.15;
    }






    @Config
    public static class TransferConstants{
        public static String leftTransferServoName = "leftTransferServo";
        public static String rightTransferServoName = "rightTransferServo";





        public static double homePosition = 0.21;
        public static double transferPosition = 0.315;




        public static long servoRiseTime = 115;
        public static long servoHomingTime = 80;
    }




    @Config
    public static class GamePadControls{
        public static GamepadEx gamepad1EX, gamepad2EX;





        public static GamepadKeys.Button cycleDriveModes = GamepadKeys.Button.DPAD_DOWN;
        public static GamepadKeys.Button enableIntake = GamepadKeys.Button.RIGHT_STICK_BUTTON;
        public static GamepadKeys.Button reverseIntake = GamepadKeys.Button.LEFT_STICK_BUTTON;
        public static GamepadKeys.Button shootArtefacts = GamepadKeys.Button.RIGHT_BUMPER;
        public static GamepadKeys.Button indexSpindexer = GamepadKeys.Button.RIGHT_BUMPER;
        public static GamepadKeys.Button selectStartingPosition = GamepadKeys.Button.RIGHT_STICK_BUTTON;





        // Autonomous Constructing Controls
        public static GamepadKeys.Button selectBlueAlliance = GamepadKeys.Button.X;
        public static GamepadKeys.Button selectRedAlliance = GamepadKeys.Button.B;
        public static GamepadKeys.Button selectAlliance = GamepadKeys.Button.A;


        public static GamepadKeys.Button selectCloseZoneStartingPosition = GamepadKeys.Button.DPAD_DOWN;
        public static GamepadKeys.Button selectFarZoneStartingPosition = GamepadKeys.Button.DPAD_UP;


        public static GamepadKeys.Button shootCloseZoneMapping = GamepadKeys.Button.A;
        public static GamepadKeys.Button shootFarZoneMapping = GamepadKeys.Button.Y;
        public static GamepadKeys.Button collectCloseSetMapping = GamepadKeys.Button.DPAD_LEFT;
        public static GamepadKeys.Button collectMiddleSetMapping = GamepadKeys.Button.DPAD_DOWN;
        public static GamepadKeys.Button collectFarSetMapping = GamepadKeys.Button.DPAD_RIGHT;
        public static GamepadKeys.Button gateCollectMapping = GamepadKeys.Button.DPAD_UP;
    }





    @Config
    public static class SensorConstants{
        public static String leftSpindexerColorSensorName = "leftSpindexerColorSensor";
        public static String rightSpindexerColorSensorName = "rightSpindexerColorSensor";





        public static String leftDistanceSensorName = "leftDistanceSensor";
        public static String rightDistanceSensorName = "rightDistanceSensor";





        public static double distanceSensorOccupiedThreshold = 9.0; //CM
        public static double leftSpindexerColorSensorOccupiedThreshold = 0.0;
        public static double rightSpindexerColorSensorOccupiedThreshold = 0.0;
    }





    @Config
    public static class AutonomousPositionConstants {

        // ── Blue Alliance Positions ──
        public static Pose blueCloseZoneStartingPosition = new Pose(12.0, 35.0, Math.toRadians(-180));
        public static Pose blueFarZoneStartingPosition = new Pose(134.0,57.0, Math.toRadians(180));
        public static Pose scoreCloseZoneBlueSide = new Pose(68.0, 61.0, Math.toRadians(-135.5));
        public static Pose scoreFarZoneBlueSide = new Pose(127.0, 62.0, Math.toRadians(-157.0));

        public static Pose gateCollectBlueSide = new Pose(89.9, 21, Math.toRadians(-124.0));

        public static Pose collectFirstSet1BlueSide = new Pose(63.5, 63.0, Math.toRadians(-90.0));
        public static Pose collectFirstSet2BlueSide = new Pose(63.5, 33.0, Math.toRadians(-90));

        public static Pose collectSecondSet1BlueSide = new Pose(89.1, 62.0, Math.toRadians(-90));
        public static Pose collectSecondSet2BlueSide = new Pose(89.1, 33.15, Math.toRadians(-90));

        public static Pose collectThirdSet1BlueSide = new Pose(109.0, 58.0, Math.toRadians(-90));
        public static Pose collectThirdSet2BlueSide = new Pose(109.0, 28.1, Math.toRadians(-90));






        // ── Red Alliance Positions ──
        public static Pose redCloseZoneStartingPosition = new Pose(12.0, 107.0, Math.toRadians(180));
        public static Pose redFarZoneStartingPosition = new Pose(135.0,88.0, Math.toRadians(180));
        public static Pose scoreCloseZoneRedSide = new Pose(48.0, 90.0, Math.toRadians(136.0));
        public static Pose scoreFarZoneRedSide   = new Pose(124.0, 85.0, Math.toRadians(159.15));

        public static Pose gateCollectRedSide = new Pose(75.5, 131.0,  Math.toRadians(125.75));

        public static Pose collectThirdSet1RedSide = new Pose(48, 100.0, Math.toRadians(90.0));
        public static Pose collectThirdSet2RedSide = new Pose(48, 129, Math.toRadians(90.0));

        public static Pose collectSecondSet1RedSide = new Pose(70, 100.0, Math.toRadians(90.0));
        public static Pose collectSecondSet2RedSide = new Pose(70, 137.15, Math.toRadians(90.0));

        public static Pose collectFirstSet1RedSide = new Pose(94.15, 100.0, Math.toRadians(90.0));
        public static Pose collectFirstSet2RedSide = new Pose(94.15, 135.15, Math.toRadians(90.0));
    }
}