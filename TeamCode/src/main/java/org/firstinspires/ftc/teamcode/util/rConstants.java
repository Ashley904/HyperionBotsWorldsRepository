package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class rConstants {
    @Config
    public static class Enums{
        public enum Alliance {BLUE, RED}
        public static Alliance selectedAlliance = Alliance.BLUE;





        public enum DriveMode {RobotCentric, FieldCentric}
        public static DriveMode selectedDriveMode = DriveMode.FieldCentric;





        public enum ShooterState {Disabled, Accelerating, TargetReached}
        public static ShooterState currentShooterState = ShooterState.Disabled;
    }





    @Config
    public static class AllianceHeadingOffsets{
        public static double blueAllianceHeadingOffset = Math.toRadians(-90);
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





        public static double driveCubicTerm=0.5, driveLinearTerm=0.4;
        public static double maximumDriveTrainSpeed=1.0, minimumDriveTrainSpeed=0.35;
    }





    @Config
    public static class IntakeConstants{
        public static String intakeMotorName = "intakeMotor";
        public static Boolean intakeInverted = false;
        public static Boolean floatModeEnabled = false;





        public static double disabledSpeed = 0.0;
        public static double intakingSpeed = 1.0;
        public static double reversingSpeed = -1.0;
        public static double idlingSpeed = 0.75;





        public static double currentAlert = 6.0;
        public static double jamVelocityAlert = 900.0;
        public static double jamTimeConfirmation = 100;
    }





    @Config
    public static class SpindexerConstants{
        public static String leftSpindexerServoName = "leftSpindexerServo";
        public static String rightSpindexerServoName = "rightSpindexerServo";
        public static String spindexerEncoderName = "spindexerEncoder";





        public static double[] intakingPositions = {0.11, 0.28, 0.452};
        public static double[] shootingPositions = {0.195, 0.365, 0.535};
        public static double[] encoderShootingPositions = {1430, 4120, 6850};




        public static double spindexerVelocityCheckDelay = 0;
        public static double minimumPeakVelocity = 2000;
        public static double decelerationTriggerRatio = 0.1;




        public static double positionalTolerance = 2390;
    }





    @Config
    public static class ShooterConstants{
        public static String leftFlyWheelMotorName = "leftFlyWheelMotor";
        public static String rightFlyWheelMotorName = "rightFlyWheelMotor";





        public static String hoodServoName = "hoodServo";






        public static boolean leftFlyWheelMotorInverted = true;
        public static boolean rightFlyWheelMotorInverted = false;
        public static boolean floatModeEnabled=false;



        public static double flyWheelKs=0.1, flyWheelKf=0.0003, flyWheelKp=0.00175;
        public static double nominalVoltage=13.35;






        public static double maximumFlyWheelVelocity=2600;
        public static double velocityReachedTolerance=30.0;





        public double minimumHoodPosition=0.0, maximumHoodPosition=0.0;
    }






    @Config
    public static class TransferConstants{
        public static String leftTransferServoName = "leftTransferServo";
        public static String rightTransferServoName = "rightTransferServo";





        public static double homePosition = 0.31;
        public static double transferPosition = 0.4;




        public static long servoRiseTime = 80;
        public static long servoHomingTime = 115;
    }




    @Config
    public static class GamePadControls{
        public static GamepadEx gamepad1EX, gamepad2EX;





        public static GamepadKeys.Button cycleDriveModes = GamepadKeys.Button.DPAD_DOWN;
        public static GamepadKeys.Button enableIntake = GamepadKeys.Button.RIGHT_STICK_BUTTON;
        public static GamepadKeys.Button reverseIntake = GamepadKeys.Button.LEFT_STICK_BUTTON;
        public static GamepadKeys.Button shootArtefacts = GamepadKeys.Button.A;
        public static GamepadKeys.Button indexSpindexer = GamepadKeys.Button.RIGHT_BUMPER;
    }

}
