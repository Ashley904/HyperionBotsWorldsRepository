package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class rConstants {
    public static class Enums{
        public enum DriveMode {RobotCentric, FieldCentric}
        public static DriveMode selectedDriveMode = DriveMode.FieldCentric;
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





        public static double disabledSpeed=0.0;
        public static double intakingSpeed=1.0;
        public static double reversingSpeed=-1.0;





        public static double currentAlert = 6.5;
        public static double jamVelocityAlert = 1000.0;
        public static double jamTimeConfirmation=100;
    }





    @Config
    public static class SpindexerConstants{
        public static String leftSpindexerServoName = "leftSpindexerServo";
        public static String rightSpindexerServoName = "rightSpindexerServo";





        public static double intakingPosition = 0.973;
        public static double lineUpForCyclePosition = 0.0;
        public static double cycleSpindexerPosition = 0.0;
    }





    @Config
    public static class GamePadControls{
        public static GamepadEx gamepad1EX, gamepad2EX;





        public static GamepadKeys.Button cycleDriveModes = GamepadKeys.Button.DPAD_DOWN;
        public static GamepadKeys.Button enableIntake = GamepadKeys.Button.RIGHT_STICK_BUTTON;
        public static GamepadKeys.Button reverseIntake = GamepadKeys.Button.LEFT_STICK_BUTTON;
    }

}
