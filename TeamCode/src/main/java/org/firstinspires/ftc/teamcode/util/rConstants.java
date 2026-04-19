package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.customPathing.Point;

public class rConstants {
    @Config
    public static class FieldConstants{
        public static double fieldSize = 144.0;






        public static double blueGoalXPosition = 134.0, blueGoalYPosition = 133.0;
        public static double redGoalXPosition = 132.0, redGoalYPosition = 8.0;





        public static double startingXPosition = 72.0, startingYPosition = 72.0;
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
        public static double blueAllianceHeadingOffset = Math.toRadians(0);
        public static double redAllianceHeadingOffset = Math.toRadians(90);
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





        public static double driveCubicTerm=0.6, driveLinearTerm=0.5;
        public static double maximumDriveTrainSpeed=1.0, minimumDriveTrainSpeed=0.35;
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
        public static double translationalKSQ = 0.0;
        public static double translationalKD = 0.0;

        public static double headingKSQ = 0.0;
        public static double headingKd = 0.0;




        public static double radiusTolerance = 0.0;
        public static double headingTolerance = 0.0; // Degrees






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
        public static double idlingSpeed = 0.4;





        public static double currentAlert = 7.0;
        public static double jamVelocityAlert = 800.0;
        public static double jamTimeConfirmation = 150;
    }





    @Config
    public static class SpindexerConstants{
        public static String leftSpindexerServoName = "leftSpindexerServo";
        public static String rightSpindexerServoName = "rightSpindexerServo";
        public static String spindexerEncoderName = DriveTrainConstants.backLeftMotorName;





        public static double[] intakingPositions = {0.075, 0.3, 0.53};
        public static double[] shootingPositions = {0.18, 0.41, 0.65};
        public static double[] encoderShootingPositions = {1360, 4100, 6900};




        public static double spindexerVelocityCheckDelay = 50;
        public static double minimumPeakVelocity = 1000;
        public static double decelerationTriggerRatio = 0.1;




        public static double positionalTolerance = 2000;
    }





    @Config
    public static class ShooterConstants{
        public static String leftFlyWheelMotorName = "leftFlyWheelMotor";
        public static String rightFlyWheelMotorName = "rightFlyWheelMotor";





        public static String hoodServoName = "hoodServo";






        public static boolean leftFlyWheelMotorInverted = false;
        public static boolean rightFlyWheelMotorInverted = true;
        public static boolean floatModeEnabled=false;



        public static double flyWheelKs=0.19, flyWheelKf=0.000365, flyWheelKp=0.0025;
        public static double nominalVoltage=13.4;






        public static double maximumFlyWheelVelocity=1900;
        public static double velocityReachedTolerance=30.0;





        public static double flywheelToBallSpeedRatio = 0.15;





        public static double minimumHoodPosition=0, maximumHoodPosition=0.775;
    }





    @Config
    public static class TurretConstants{
        public static String turretMotorName = "turretMotor";
        public static boolean turretMotorInverted = false;
        public static boolean floatModeEnabled = false;





        public static double turretKp=0.004, turretKd=0.325;
        public static double maxTurretPower=0.9;





        public static double turretMaxLeftAngle = 125, turretMaxRightAngle = -125;
        public static double turretOffset = 5;
    }






    @Config
    public static class TransferConstants{
        public static String leftTransferServoName = "leftTransferServo";
        public static String rightTransferServoName = "rightTransferServo";





        public static double homePosition = 0.44;
        public static double transferPosition = 0.3;




        public static long servoRiseTime = 110;
        public static long servoHomingTime = 75;
    }




    @Config
    public static class GamePadControls{
        public static GamepadEx gamepad1EX, gamepad2EX;





        public static GamepadKeys.Button cycleDriveModes = GamepadKeys.Button.DPAD_DOWN;
        public static GamepadKeys.Button enableIntake = GamepadKeys.Button.RIGHT_STICK_BUTTON;
        public static GamepadKeys.Button reverseIntake = GamepadKeys.Button.LEFT_STICK_BUTTON;
        public static GamepadKeys.Button shootArtefacts = GamepadKeys.Button.A;
        public static GamepadKeys.Button indexSpindexer = GamepadKeys.Button.RIGHT_BUMPER;





        // Autonomous Constructing Controls
        public static GamepadKeys.Button selectBlueAlliance = GamepadKeys.Button.X;
        public static GamepadKeys.Button selectRedAlliance = GamepadKeys.Button.B;


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





        public static double distanceSensorOccupiedThreshold = 10.0; //CM
        public static double leftSpindexerColorSensorOccupiedThreshold = 0.0;
        public static double rightSpindexerColorSensorOccupiedThreshold = 0.0;
    }





    @Config
    public static class AutonomousPositionConstants {

        // ── Blue Alliance Positions ──
        public static Point scoreCloseZoneBlueSide = new Point(0, 0, 0);
        public static Point scoreFarZoneBlueSide   = new Point(0, 0, 0);

        public static Point gateCollectBlueSide = new Point(0, 0, 0);

        public static Point collectThirdSet1BlueSide = new Point(0, 0, 0);
        public static Point collectThirdSet2BlueSide = new Point(0, 0, 0);

        public static Point collectSecondSet1BlueSide = new Point(0, 0, 0);
        public static Point collectSecondSet2BlueSide = new Point(0, 0, 0);

        public static Point collectFirstSet1BlueSide = new Point(0, 0, 0);
        public static Point collectFirstSet2BlueSide = new Point(0, 0, 0);


        // ── Red Alliance Positions ──
        public static Point scoreCloseZoneRedSide = new Point(0, 0, 0);
        public static Point scoreFarZoneRedSide   = new Point(0, 0, 0);

        public static Point gateCollectRedSide = new Point(0, 0, 0);

        public static Point collectThirdSet1RedSide = new Point(0, 0, 0);
        public static Point collectThirdSet2RedSide = new Point(0, 0, 0);

        public static Point collectSecondSet1RedSide = new Point(0, 0, 0);
        public static Point collectSecondSet2RedSide = new Point(0, 0, 0);

        public static Point collectFirstSet1RedSide = new Point(0, 0, 0);
        public static Point collectFirstSet2RedSide = new Point(0, 0, 0);
    }
}
