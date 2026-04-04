package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class InitializeSpindexerCMD extends SequentialCommandGroup{
    private final RobotHardwareMap robot;




    public InitializeSpindexerCMD(RobotHardwareMap robot, SpindexerSubsystem spindexerSubsystem){
        this.robot = robot;

        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(() -> spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[0])),
                        new WaitCommand(1000),
                        new InstantCommand(() -> robot.spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER))
                )
        );
    }

}
