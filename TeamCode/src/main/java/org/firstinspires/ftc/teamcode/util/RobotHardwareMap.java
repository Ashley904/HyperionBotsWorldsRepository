package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpointDriver;

public class RobotHardwareMap {
    public DcMotorEx front_left_motor, back_left_motor, front_right_motor, back_right_motor;
    public DcMotorEx intakeMotor;





    public DcMotorEx turretMotor;





    public Servo leftSpindexerServo, rightSpindexerServo;
    public DcMotorEx spindexerEncoder;





    public DcMotorEx leftFlyWheelMotor, rightFlyWheelMotor;
    public Servo hoodServo;





    public GoBildaPinpointDriver pinpointDriver;





    public Servo leftTransferServo;
    public Servo rightTransferServo;





    public ColorRangeSensor leftSpindexerColorSensor;
    public ColorRangeSensor rightSpindexerColorSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;





    public HardwareMap hardwareMap;





    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        initializeDriveTrain(hardwareMap);
        initializeTurret(hardwareMap);
        initializeIntake(hardwareMap);
        initializeShooter(hardwareMap);
        initializeTransfer(hardwareMap);
        initializeSpindexer(hardwareMap);
        initializeColorSensors(hardwareMap);
        initializePinpointDriver(hardwareMap);
        initializeDistanceSensors(hardwareMap);

        // Share the back left motor reference for spindexer encoder reading
        spindexerEncoder = back_left_motor;
        spindexerEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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





    private void initializeTurret(HardwareMap hwMap){
        turretMotor = hwMap.get(DcMotorEx.class, rConstants.TurretConstants.turretMotorName);
        turretMotor.setDirection(rConstants.TurretConstants.turretMotorInverted ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(rConstants.TurretConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }





    private void initializeIntake(HardwareMap hwMap){
        intakeMotor = getMotor(hwMap, rConstants.IntakeConstants.intakeMotorName, rConstants.IntakeConstants.intakeInverted, rConstants.IntakeConstants.floatModeEnabled);

        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setCurrentAlert(rConstants.IntakeConstants.currentAlert, CurrentUnit.AMPS);
    }





    private void initializeSpindexer(HardwareMap hardwareMap){
        // No separate encoder init here — shared reference is set in init()
        leftSpindexerServo = hardwareMap.get(Servo.class, rConstants.SpindexerConstants.leftSpindexerServoName);
        rightSpindexerServo = hardwareMap.get(Servo.class, rConstants.SpindexerConstants.rightSpindexerServoName);
    }





    private void initializeShooter(HardwareMap hardwareMap){
        leftFlyWheelMotor = getMotor(hardwareMap, rConstants.ShooterConstants.leftFlyWheelMotorName, rConstants.ShooterConstants.leftFlyWheelMotorInverted, rConstants.ShooterConstants.floatModeEnabled);
        rightFlyWheelMotor = getMotor(hardwareMap, rConstants.ShooterConstants.rightFlyWheelMotorName, rConstants.ShooterConstants.rightFlyWheelMotorInverted, rConstants.ShooterConstants.floatModeEnabled);

        hoodServo = hardwareMap.get(Servo.class, rConstants.ShooterConstants.hoodServoName);
    }





    private void initializeTransfer(HardwareMap hardwareMap){
        leftTransferServo = hardwareMap.get(Servo.class, rConstants.TransferConstants.leftTransferServoName);
        leftTransferServo.setDirection(Servo.Direction.REVERSE);

        rightTransferServo = hardwareMap.get(Servo.class, rConstants.TransferConstants.rightTransferServoName);
        rightTransferServo.setDirection(Servo.Direction.FORWARD);
    }





    public void initializePinpointDriver(HardwareMap hardwareMap){
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, rConstants.PinpointConstants.pinpointDriveName);
        pinpointDriver.setEncoderDirections(rConstants.PinpointConstants.forwardPodInverted ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                rConstants.PinpointConstants.strafePodInverted ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpointDriver.setOffsets(rConstants.PinpointConstants.strafePodOffset, rConstants.PinpointConstants.forwardPodOffset, DistanceUnit.INCH);
    }





    public void initializeColorSensors(HardwareMap hardwareMap){
        leftSpindexerColorSensor = hardwareMap.get(ColorRangeSensor.class, rConstants.SensorConstants.leftSpindexerColorSensorName);
        rightSpindexerColorSensor = hardwareMap.get(ColorRangeSensor.class, rConstants.SensorConstants.rightSpindexerColorSensorName);
    }





    public void initializeDistanceSensors(HardwareMap hardwareMap){
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, rConstants.SensorConstants.leftDistanceSensorName);
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, rConstants.SensorConstants.rightDistanceSensorName);
    }
}