package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final ElapsedTime intakeJamTimer, intakeUnjamTimer;
    private final RobotHardwareMap robot;





    public enum IntakeState {Disabled, Intaking, Jammed, Reversing}
    IntakeState intakeState = IntakeState.Disabled;





    public IntakeSubsystem(RobotHardwareMap robotHardwareMap){
        this.robot = robotHardwareMap;
        intakeJamTimer = new ElapsedTime();
        intakeUnjamTimer = new ElapsedTime();
    }





    private boolean jamTimerRunning = false;
    private boolean unjamTimerRunning = false;





    @Override
    public void periodic(){
        IntakeSpeedControl();
        JammingControl();
    }





    private void IntakeSpeedControl(){
        switch(intakeState){
            case Disabled:
                robot.intakeMotor.setPower(rConstants.IntakeConstants.disabledSpeed);
                break;

            case Intaking:
                robot.intakeMotor.setPower(rConstants.IntakeConstants.intakingSpeed);
                break;

            case Jammed:
                intakeState = IntakeState.Reversing;
                break;

            case Reversing:
                robot.intakeMotor.setPower(rConstants.IntakeConstants.reversingSpeed);
                break;
        }
    }
    private void JammingControl(){

        // Jam detection - only while intaking
        if(intakeState == IntakeState.Intaking){
            boolean currentSpikeDetected = robot.intakeMotor.isOverCurrent();
            boolean velocityDropDetected = Math.abs(robot.intakeMotor.getVelocity()) <= rConstants.IntakeConstants.jamVelocityAlert;

            if(currentSpikeDetected && velocityDropDetected){
                if(!jamTimerRunning){
                    intakeJamTimer.reset();
                    jamTimerRunning = true;
                } else if(intakeJamTimer.milliseconds() >= rConstants.IntakeConstants.jamTimeConfirmation){
                    intakeState = IntakeState.Jammed;
                    jamTimerRunning = false;
                }
            } else {
                jamTimerRunning = false;
            }
        }

        // Unjam detection - while reversing
        if(intakeState == IntakeState.Reversing){
            boolean currentNormal = !robot.intakeMotor.isOverCurrent();
            boolean velocityNormal = Math.abs(robot.intakeMotor.getVelocity()) > rConstants.IntakeConstants.jamVelocityAlert;

            if(currentNormal && velocityNormal){
                if(!unjamTimerRunning){
                    intakeUnjamTimer.reset();
                    unjamTimerRunning = true;
                } else if(intakeUnjamTimer.milliseconds() >= rConstants.IntakeConstants.jamTimeConfirmation){
                    intakeState = IntakeState.Intaking;
                    unjamTimerRunning = false;
                }
            } else {
                unjamTimerRunning = false;
            }
        }
    }





    // Helper functions
    public void setState(IntakeState state) { intakeState = state; }
    public IntakeState getState() { return intakeState; }
    public double getIntakeVelocity() { return robot.intakeMotor.getVelocity(); }
    public double getIntakeCurrentDraw() { return robot.intakeMotor.getCurrent(CurrentUnit.AMPS); }
}