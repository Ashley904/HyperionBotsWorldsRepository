package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class SpindexerSubsystem extends SubsystemBase{
    public enum SpindexerState {Intaking, Shooting}
    public static SpindexerState currentSpindexerState = SpindexerState.Intaking;





    private final ElapsedTime spindexerStartDelayTimer = new ElapsedTime();
    private final ElapsedTime spindexerVelocityTimer = new ElapsedTime();





    private boolean spindexerStartDelayActive = false;
    private boolean spindexerVelocityIsZeroed = true;
    private boolean spindexerVelocityTimerRunning = false;
    private boolean hasPeaked = false;
    private double peakVelocity = 0;






    private final RobotHardwareMap robot;




    private double previousSpindexerPosition = 0.0;
    private double cachedEncoderVelocity, cachedEncoderPosition;




    private double targetEncoderPosition = 0;
    private boolean positionCheckActive = false;
    private boolean positionReached = false;




    private int currentIntakingIndex = 0;





    public SpindexerSubsystem(RobotHardwareMap robotHardwareMap){
        this.robot = robotHardwareMap;
    }





    @Override
    public void periodic(){
        cachedEncoderVelocity = robot.spindexerEncoder.getVelocity();
        cachedEncoderPosition = Math.abs(robot.spindexerEncoder.getCurrentPosition());

        //Calling Functions
        updateSpindexerVelocityZeroedStatus();
        updatePositionCheck();
    }





    public void setSpindexerPosition(double spindexerPosition) {
        robot.leftSpindexerServo.setPosition(spindexerPosition); robot.rightSpindexerServo.setPosition(spindexerPosition);

        if(spindexerPosition != previousSpindexerPosition){
            previousSpindexerPosition = spindexerPosition;
            startSpindexerCheck();
        }
    }
    public void setSpindexerState(SpindexerState state) { currentSpindexerState = state; }





    // Helpers
    public SpindexerState getSpindexerState() { return currentSpindexerState; }
    public double getVelocity() { return cachedEncoderVelocity; }
    public double getPosition() { return Math.abs(cachedEncoderPosition); }







    // Spindexer Velocity Checks
    private void startSpindexerCheck(){
        spindexerStartDelayTimer.reset();
        spindexerStartDelayActive = true;
        spindexerVelocityIsZeroed = false;
        hasPeaked = false;
        peakVelocity = 0;
    }

    private void updateSpindexerVelocityZeroedStatus(){
        if(!spindexerStartDelayActive || spindexerStartDelayTimer.milliseconds() < rConstants.SpindexerConstants.spindexerVelocityCheckDelay) return;
        double currentVelocity = Math.abs(cachedEncoderVelocity);

        if(currentVelocity > peakVelocity){
            peakVelocity = currentVelocity;
        } else if(peakVelocity > rConstants.SpindexerConstants.minimumPeakVelocity
                && currentVelocity < peakVelocity * rConstants.SpindexerConstants.decelerationTriggerRatio){
            spindexerVelocityIsZeroed = true;
        }
    }
    public boolean spindexerPositionReached() { return spindexerVelocityIsZeroed; }





    // Positional Spindexer Position Checking
    public void startPositionCheck(int positionIndex) {
        targetEncoderPosition = rConstants.SpindexerConstants.encoderShootingPositions[positionIndex];
        positionCheckActive = true;
        positionReached = false;
    }

    private void updatePositionCheck() {
        if (!positionCheckActive) return;

        double currentPosition = Math.abs(robot.spindexerEncoder.getCurrentPosition());
        if (Math.abs(currentPosition - targetEncoderPosition) <= rConstants.SpindexerConstants.positionalTolerance) {
            positionReached = true;
            positionCheckActive = false;
        }
    }
    public boolean spindexerPositionalReached() { return positionReached; }





    public void cycleIntakingPosition() {
        currentIntakingIndex++;
        if (currentIntakingIndex > 2) currentIntakingIndex = 0;
        setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[currentIntakingIndex]);
    }
    public int getCurrentIntakingIndex() { return currentIntakingIndex; }

    public void resetIntakingIndex() { currentIntakingIndex = 0; }
}