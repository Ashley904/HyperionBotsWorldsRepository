package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class SpindexerSubsystem extends SubsystemBase{
    public enum SpindexerState {Intaking, LinedUpForCycle, CyclingArtefacts}
    public static SpindexerState currentSpindexerState = SpindexerState.Intaking;





    private final RobotHardwareMap robot;





    public SpindexerSubsystem(RobotHardwareMap robotHardwareMap){
        this.robot = robotHardwareMap;
    }





    @Override
    public void periodic(){
        //Calling Functions
        SpindexerPositionControl();
    }





    private void SpindexerPositionControl(){
        switch(currentSpindexerState){
            case Intaking:
                setSpindexerPosition(rConstants.SpindexerConstants.intakingPosition);
                break;

            case LinedUpForCycle:
                setSpindexerPosition(rConstants.SpindexerConstants.lineUpForCyclePosition);
                break;

            case CyclingArtefacts:
                setSpindexerPosition(rConstants.SpindexerConstants.cycleSpindexerPosition);
        }
    }





    private void setSpindexerPosition(double position) { robot.leftSpindexerServo.setPosition(position); robot.rightSpindexerServo.setPosition(position); }





    //Helpers
    public SpindexerState getSpindexerState() { return currentSpindexerState; }
    public void setSpindexerState(SpindexerState state) { currentSpindexerState = state;}
}
