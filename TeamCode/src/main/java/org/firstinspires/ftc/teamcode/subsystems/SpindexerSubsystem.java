package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class SpindexerSubsystem extends SubsystemBase{
    public enum SpindexerState {Intaking, LineUpForCycle, CyclingArtefacts}
    public static SpindexerState currentSpindexerState = SpindexerState.Intaking;





    private final ElapsedTime spindexerStartDelayTimer = new ElapsedTime();
    private final ElapsedTime spindexerVelocityTimer = new ElapsedTime();

    private boolean spindexerStartDelayActive = false;
    private boolean spindexerVelocityIsZeroed = false;
    private boolean spindexerVelocityTimerRunning = false;






    private final RobotHardwareMap robot;





    public SpindexerSubsystem(RobotHardwareMap robotHardwareMap){
        this.robot = robotHardwareMap;
    }





    @Override
    public void periodic(){
        //Calling Functions
        spindexerPositionControl();
        updateSpindexerVelocityZeroedStatus();
    }





    private void spindexerPositionControl(){
        switch(currentSpindexerState){
            case Intaking:
                setSpindexerPosition(rConstants.SpindexerConstants.intakingPosition);
                break;

            case LineUpForCycle:
                setSpindexerPosition(rConstants.SpindexerConstants.lineUpForCyclePosition);
                break;

            case CyclingArtefacts:
                setSpindexerPosition(rConstants.SpindexerConstants.cycleSpindexerPosition);
        }
    }





    private void setSpindexerPosition(double position) { robot.leftSpindexerServo.setPosition(position); robot.rightSpindexerServo.setPosition(position); }





    // Helpers
    public SpindexerState getSpindexerState() { return currentSpindexerState; }
    public void setSpindexerState(SpindexerState state) { currentSpindexerState = state; startSpindexerCheck(); }
    public double getVelocity() { return robot.spindexerEncoder.getVelocity(); }







    // Spindexer Velocity Checks
    private void startSpindexerCheck(){
        spindexerStartDelayTimer.reset();
        spindexerStartDelayActive = true;
        spindexerVelocityIsZeroed = false;
        spindexerVelocityTimerRunning = false;
    }
    private void updateSpindexerVelocityZeroedStatus(){
        if(!spindexerStartDelayActive || spindexerStartDelayTimer.milliseconds() < rConstants.SpindexerConstants.spindexerVelocityCheckDelay) return; //Velocity Check Delay

        if(robot.spindexerEncoder.getVelocity() <= rConstants.SpindexerConstants.spindexerVelocityZeroThreshold){
            if(!spindexerVelocityTimerRunning){
                spindexerVelocityTimer.reset();
                spindexerVelocityTimerRunning = true;
            } else if(spindexerVelocityTimer.milliseconds() >= 100) { spindexerVelocityIsZeroed = true; }
        }else {
            spindexerVelocityTimerRunning = false;
            spindexerVelocityIsZeroed = false;
        }
    }
    public boolean spindexerPositionReached() { return spindexerVelocityIsZeroed; }
}
