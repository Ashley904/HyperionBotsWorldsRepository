package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.customPathing.Point;

public class rConstants {
    @Config
    public static class FieldConstants{
        public static Pose blueGoalPose = new Pose(136.0, 133.0);
        public static Pose redGoalPose = new Pose(131.0, 9.0);





        public static Pose startingPose = new Pose(72.0, 72.0);
    }





    @Config
    public static class Enums{
        public enum Alliance {BLUE, RED}
        public static Alliance selectedAlliance = Alliance.RED;





        public enum DriveMode {RobotCentric, FieldCentric}
        public static DriveMode selectedDriveMode = DriveMode.FieldCentric;





        public enum ShooterState {Disabled, Accelerating, TargetReached}
        public static ShooterState currentShooterState = ShooterState.Disabled;
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
        public static double maximumDriveTrainSpeed=1.0, minimumDriveTrainSpeed=0.525;
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





        public static double disabledSpeed = 0.0;
        public static double intakingSpeed = 1.0;
        public static double reversingSpeed = -1.0;
        public static double idlingSpeed = 0.9;





        public static double currentAlert = 5.0;
        public static double jamVelocityAlert = 300.0;
        public static double jamTimeConfirmation = 100;
    }





    @Config
    public static class SpindexerConstants{
        public static String leftSpindexerServoName = "leftSpindexerServo";
        public static String rightSpindexerServoName = "rightSpindexerServo";
        public static String spindexerEncoderName = DriveTrainConstants.backLeftMotorName;





        public static double[] intakingPositions = {0.21, 0.43, 0.68};
        public static double[] shootingPositions = {0.30, 0.57, 0.801};
        public static double[] encoderShootingPositions = {1184, 4153, 6742};




        public static double spindexerVelocityCheckDelay = 10;
        public static double minimumPeakVelocity = 1000;
        public static double decelerationTriggerRatio = 0.25;




        public static double positionalTolerance = 1200;
        public static double positionalPercentageTolerance = 0.275;
    }





    @Config
    public static class ShooterConstants{
        public static String leftFlyWheelMotorName = "leftFlyWheelMotor";
        public static String rightFlyWheelMotorName = "rightFlyWheelMotor";





        public static String hoodServoName = "hoodServo";






        public static boolean leftFlyWheelMotorInverted = false;
        public static boolean rightFlyWheelMotorInverted = true;
        public static boolean floatModeEnabled=false;



        public static double flyWheelKs=0.15, flyWheelKf=0.00047, flyWheelKp=0.0018;
        public static double nominalVoltage=13.4;






        public static double maximumFlyWheelVelocity=3000;
        public static double velocityReachedTolerance=35.0;





        public static double flywheelToBallSpeedRatio = 0.15;





        public static double minimumHoodPosition=0.25, maximumHoodPosition=0.665;
    }





    @Config
    public static class TurretConstants{
        public static String turretMotorName = "turretMotor";
        public static boolean turretMotorInverted = false;
        public static boolean floatModeEnabled = false;





        public static double turretKp=0.025, turretKd=0.35;
        public static double maxTurretPower=0.75;





        public static double turretMinAngle = 105, turretMaxAngle = 255;
        public static double turretOffset = 0;
    }






    @Config
    public static class TransferConstants{
        public static String leftTransferServoName = "leftTransferServo";
        public static String rightTransferServoName = "rightTransferServo";





        public static double homePosition = 0.415;
        public static double transferPosition = 0.31;




        public static long servoRiseTime = 180;
        public static long servoHomingTime = 40;
    }




    @Config
    public static class GamePadControls{
        public static GamepadEx gamepad1EX, gamepad2EX;





        public static GamepadKeys.Button cycleDriveModes = GamepadKeys.Button.DPAD_DOWN;
        public static GamepadKeys.Button enableIntake = GamepadKeys.Button.RIGHT_STICK_BUTTON;
        public static GamepadKeys.Button reverseIntake = GamepadKeys.Button.LEFT_STICK_BUTTON;
        public static GamepadKeys.Button shootArtefacts = GamepadKeys.Button.RIGHT_BUMPER;
        public static GamepadKeys.Button indexSpindexer = GamepadKeys.Button.RIGHT_BUMPER;





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
        public static Point blueCloseZoneStartingPosition = new Point(131.0, 109.0, 0.0);
        public static Point scoreCloseZoneBlueSide = new Point(79.0, 82.0, 43.0);
        public static Point scoreFarZoneBlueSide   = new Point(12.0, 82.0, 23.0);

        public static Point gateCollectBlueSide = new Point(0, 0, 0);

        public static Point collectThirdSet1BlueSide = new Point(30.0, 83.0, 90.0);
        public static Point collectThirdSet2BlueSide = new Point(30.0, 114.0, 90.0);

        public static Point collectSecondSet1BlueSide = new Point(53.0, 85.0, 90.0);
        public static Point collectSecondSet2BlueSide = new Point(53.0, 114.0, 90.0);

        public static Point collectFirstSet1BlueSide = new Point(78.0, 85.0, 90.0);
        public static Point collectFirstSet2BlueSide = new Point(78.0, 113.0, 90.0);

        // ── Red Alliance Positions ──
        public static Point redCloseZoneStartingPosition = new Point(9, 106, -180);
        public static Point scoreCloseZoneRedSide = new Point(47.0, 85.0, 135.0);
        public static Point scoreFarZoneRedSide   = new Point(126.0, 89.0, 161.0);

        public static Point gateCollectRedSide = new Point(73, 129.25,  120.0);

        public static Point collectThirdSet1RedSide = new Point(44.0, 103.0, 90.0);
        public static Point collectThirdSet2RedSide = new Point(44.0, 130.0, 90.0);

        public static Point collectSecondSet1RedSide = new Point(69.15, 100.0, 90.0);
        public static Point collectSecondSet2RedSide = new Point(69.15, 135.0, 90.0);

        public static Point collectFirstSet1RedSide = new Point(92.0, 104.0, 90.0);
        public static Point collectFirstSet2RedSide = new Point(92.0, 135.0, 90.0);
    }
}
