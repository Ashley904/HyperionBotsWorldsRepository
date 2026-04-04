package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.util.rConstants;

public class TransferCMD extends SequentialCommandGroup{
    public TransferCMD(RobotHardwareMap robot, SpindexerSubsystem spindexerSubsystem, int nextPositionIndex){
        addCommands(
                new InstantCommand(() -> {
                    robot.leftTransferServo.setPosition(rConstants.TransferConstants.transferPosition);
                    robot.rightTransferServo.setPosition(rConstants.TransferConstants.transferPosition);
                }),
                new WaitCommand(rConstants.TransferConstants.servoRiseTime),
                new InstantCommand(() -> {
                    robot.leftTransferServo.setPosition(rConstants.TransferConstants.homePosition);
                    robot.rightTransferServo.setPosition(rConstants.TransferConstants.homePosition);
                }),
                new WaitCommand(rConstants.TransferConstants.servoHomingTime),
                new InstantCommand(() -> {
                    spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.shootingPositions[nextPositionIndex]);
                    spindexerSubsystem.startPositionCheck(nextPositionIndex);
                })
        );
    }
}