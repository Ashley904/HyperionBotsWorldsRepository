package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.util.rConstants;

public class InitializeTransferCMD extends SequentialCommandGroup{
    private final RobotHardwareMap robot;




    public InitializeTransferCMD(RobotHardwareMap robot){
        this.robot = robot;

        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.leftTransferServo.setPosition(rConstants.TransferConstants.homePosition)).alongWith(
                                new InstantCommand(() -> robot.rightTransferServo.setPosition(rConstants.TransferConstants.homePosition))
                        )
                )
        );
    }

}
