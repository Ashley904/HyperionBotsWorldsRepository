package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RobotHardwareMap {
    public DcMotorEx front_left_motor, back_left_motor, front_right_motor, back_right_motor;
    public DcMotorEx intakeMotor;





    public void init(HardwareMap hardwareMap){
        initializeIntake(hardwareMap);
        initializeDriveTrain(hardwareMap);
    }





    private void initializeDriveTrain(HardwareMap hwMap){
        front_left_motor = getMotor(hwMap, rConstants.DriveTrainConstants.frontLeftMotorName, rConstants.DriveTrainConstants.frontLeftInverted, rConstants.DriveTrainConstants.floatModeEnabled);
        back_left_motor = getMotor(hwMap, rConstants.DriveTrainConstants.backLeftMotorName, rConstants.DriveTrainConstants.backLeftInverted, rConstants.DriveTrainConstants.floatModeEnabled);
        front_right_motor = getMotor(hwMap, rConstants.DriveTrainConstants.frontRightMotorName, rConstants.DriveTrainConstants.frontRightInverted, rConstants.DriveTrainConstants.floatModeEnabled);
        back_right_motor = getMotor(hwMap, rConstants.DriveTrainConstants.backRightMotorName, rConstants.DriveTrainConstants.backRightInverted, rConstants.DriveTrainConstants.floatModeEnabled);
    }
    private DcMotorEx getMotor(HardwareMap hardwareMap, String motorName, Boolean inverted, Boolean floatModeEnabled){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(inverted ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(floatModeEnabled ? DcMotorEx.ZeroPowerBehavior.FLOAT : DcMotorEx.ZeroPowerBehavior.BRAKE);

        return motor;
    }





    private void initializeIntake(HardwareMap hwMap){
        intakeMotor = getMotor(hwMap, rConstants.IntakeConstants.intakeMotorName, rConstants.IntakeConstants.intakeInverted, rConstants.IntakeConstants.floatModeEnabled);

        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setCurrentAlert(rConstants.IntakeConstants.currentAlert, CurrentUnit.AMPS);
    }
}
