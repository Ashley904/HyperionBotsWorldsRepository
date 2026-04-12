package org.firstinspires.ftc.teamcode.customPathing;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controllers.SQUIDController;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class Drive {
    RobotHardwareMap robot;





    private Point targetPoint;





    private Vector2d errorVector = new Vector2d();





    private SQUIDController headingSQUIDController = new SQUIDController();
    private SQUIDController translationalSQUIDController = new SQUIDController();





    private double headingErrorInRadians = 0.0;




    double drive=0.0, turn=0.0, strafe=0.0;





    private ElapsedTime targetConfirmationTime = new ElapsedTime();
    private ElapsedTime overTimeProtectionTimer = new ElapsedTime();





    public void driveToTargetPoint(double currentRobotHeading, double currentRobotXPosition, double currentRobotYPosition){
        // Error Calculations
        headingErrorInRadians = AngleUnit.normalizeRadians(Math.toRadians(targetPoint.getHeading()) - Math.toRadians(currentRobotHeading));
        errorVector = new Vector2d(targetPoint.getX() - currentRobotXPosition, targetPoint.getY() - currentRobotYPosition);

        errorVector = errorVector.rotateBy(-currentRobotHeading);

        // Calculating Powers
        turn = -headingSQUIDController.calculate(headingErrorInRadians, rConstants.AutonomousConstants.headingKSQ, rConstants.AutonomousConstants.headingKd);
        drive = translationalSQUIDController.calculate(errorVector.getX(), rConstants.AutonomousConstants.translationalKSQ, rConstants.AutonomousConstants.translationalKD);
        strafe = -translationalSQUIDController.calculate(errorVector.getY(), rConstants.AutonomousConstants.translationalKSQ, rConstants.AutonomousConstants.translationalKD);
    }





    public boolean isInRadius(double currentRobotXPosition, double currentRobotYPosition){
        double xDelta = targetPoint.getX() - currentRobotXPosition;
        double yDelta = targetPoint.getY() - currentRobotYPosition;
        double distanceSquared = (xDelta * xDelta) + (yDelta * yDelta);

        return distanceSquared <= rConstants.AutonomousConstants.radiusTolerance;
    }

    public void trajectoryStartSequence() {targetConfirmationTime.reset(); overTimeProtectionTimer.reset(); }

    public boolean isAtTarget(double currentRobotXPosition, double currentRobotYPosition){
        if(!isInRadius(currentRobotXPosition, currentRobotYPosition)
                || Math.abs(Math.toDegrees(headingErrorInRadians)) >= rConstants.AutonomousConstants.headingTolerance){
            targetConfirmationTime.reset();
        }

        return targetConfirmationTime.seconds() >= rConstants.AutonomousConstants.targetConfirmationSeconds ||
                overTimeProtectionTimer.seconds() >= rConstants.AutonomousConstants.overtimeProtectionSeconds;
    }





    private void setDriveTrainMotorPowers(double drive, double strafe, double turn){
        double frontLeftMotorPower = drive + strafe + turn;
        double backLeftMotorPower = drive - strafe - turn;
        double frontRightMotorPower = drive - strafe + turn;
        double backRightMotorPower = drive + strafe - turn;

        double maxPower = Math.max(Math.abs(frontLeftMotorPower), Math.max(Math.abs(frontRightMotorPower),
                Math.max(Math.abs(backLeftMotorPower), Math.abs(backRightMotorPower))));
        if(maxPower > 1.0){
            frontLeftMotorPower /= maxPower;
            backLeftMotorPower /= maxPower;
            frontRightMotorPower /= maxPower;
            backRightMotorPower /= maxPower;
        }

        robot.front_left_motor.setPower(frontLeftMotorPower);
        robot.back_left_motor.setPower(backLeftMotorPower);
        robot.front_right_motor.setPower(frontRightMotorPower);
        robot.back_right_motor.setPower(backRightMotorPower);
    }




    public void stopMotor(){
        robot.front_left_motor.setPower(0);
        robot.back_left_motor.setPower(0);
        robot.front_right_motor.setPower(0);
        robot.back_right_motor.setPower(0);
    }
}
