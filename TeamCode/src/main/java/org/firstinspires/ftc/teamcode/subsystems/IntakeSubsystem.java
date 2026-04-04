package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final ElapsedTime intakeJamTimer, intakeUnjamTimer;
    private final RobotHardwareMap robot;





    public enum IntakeState {Disabled, Intaking, Reversing, IntakingUnJamming, OuttakingUnJamming, Idling}
    IntakeState intakeState = IntakeState.Disabled;





    private double cachedVelocity;
    private double cachedCurrent;
    private boolean cachedOverCurrent;





    public IntakeSubsystem(RobotHardwareMap robotHardwareMap){
        this.robot = robotHardwareMap;
        intakeJamTimer = new ElapsedTime();
        intakeUnjamTimer = new ElapsedTime();
    }





    private boolean jamTimerRunning = false;
    private boolean unjamTimerRunning = false;





    @Override
    public void periodic(){
        cachedVelocity = robot.intakeMotor.getVelocity();
        cachedCurrent = robot.intakeMotor.getCurrent(CurrentUnit.AMPS);
        cachedOverCurrent = robot.intakeMotor.isOverCurrent();

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

            case IntakingUnJamming:
            case Reversing:
                robot.intakeMotor.setPower(rConstants.IntakeConstants.reversingSpeed);
                break;

            case OuttakingUnJamming:
                robot.intakeMotor.setPower(rConstants.IntakeConstants.reversingSpeed * -1);
                break;

            case Idling:
                robot.intakeMotor.setPower(rConstants.IntakeConstants.idlingSpeed);
        }
    }






    private void JammingControl(){
        // Jam Detection - While Intaking
        if(intakeState == IntakeState.Intaking){
            boolean currentSpikeDetected = robot.intakeMotor.isOverCurrent();
            boolean velocityDropDetected = Math.abs(robot.intakeMotor.getVelocity()) <= rConstants.IntakeConstants.jamVelocityAlert;

            if(currentSpikeDetected && velocityDropDetected){
                if(!jamTimerRunning){
                    intakeJamTimer.reset();
                    jamTimerRunning = true;
                } else if(intakeJamTimer.milliseconds() >= rConstants.IntakeConstants.jamTimeConfirmation){
                    intakeState = IntakeState.IntakingUnJamming;
                    jamTimerRunning = false;
                }
            } else {
                jamTimerRunning = false;
            }
        }
        // IntakingUnJamming Condition
        if(intakeState == IntakeState.IntakingUnJamming){
            boolean currentNormal = !robot.intakeMotor.isOverCurrent();
            boolean velocityNormal = Math.abs(robot.intakeMotor.getVelocity()) > rConstants.IntakeConstants.jamVelocityAlert;

            if(currentNormal && velocityNormal){
                if(!unjamTimerRunning){ intakeUnjamTimer.reset(); unjamTimerRunning = true; }
                else if(intakeUnjamTimer.milliseconds() >= rConstants.IntakeConstants.jamTimeConfirmation){
                    intakeState = IntakeState.Intaking;
                    unjamTimerRunning = false;
                }
            } else { unjamTimerRunning = false; }
        }





        // Jam Detection - While Outtaking
        if(intakeState == IntakeState.Reversing){
            boolean currentSpikeDetected = robot.intakeMotor.isOverCurrent();
            boolean velocityDropDetected = Math.abs(robot.intakeMotor.getVelocity()) <= rConstants.IntakeConstants.jamVelocityAlert;

            if(currentSpikeDetected && velocityDropDetected){
                if(!jamTimerRunning){
                    intakeJamTimer.reset();
                    jamTimerRunning = true;
                } else if(intakeJamTimer.milliseconds() >= rConstants.IntakeConstants.jamTimeConfirmation){
                    intakeState = IntakeState.OuttakingUnJamming;
                    jamTimerRunning = false;
                }
            } else {
                jamTimerRunning = false;
            }
        }
        // OuttakingUnJamming Condition
        if(intakeState == IntakeState.OuttakingUnJamming){
            boolean currentNormal = !robot.intakeMotor.isOverCurrent();
            boolean velocityNormal = Math.abs(robot.intakeMotor.getVelocity()) > rConstants.IntakeConstants.jamVelocityAlert;

            if(currentNormal && velocityNormal){
                if(!unjamTimerRunning){ intakeUnjamTimer.reset(); unjamTimerRunning = true; }
                else if(intakeUnjamTimer.milliseconds() >= rConstants.IntakeConstants.jamTimeConfirmation){
                    intakeState = IntakeState.Reversing;
                    unjamTimerRunning = false;
                }
            } else { unjamTimerRunning = false; }
        }
    }





    // Helper functions
    public void setState(IntakeState state) { intakeState = state; }
    public IntakeState getState() { return intakeState; }
    public double getIntakeVelocity() { return cachedVelocity; }
    public double getIntakeCurrentDraw() { return cachedCurrent; }
}