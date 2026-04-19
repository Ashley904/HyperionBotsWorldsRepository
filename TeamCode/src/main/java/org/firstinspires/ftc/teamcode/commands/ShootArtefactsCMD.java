package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class ShootArtefactsCMD extends SequentialCommandGroup {
    private final IntakeSubsystem intakeSubsystem;
    private final SpindexerSubsystem spindexerSubsystem;

    public ShootArtefactsCMD(SpindexerSubsystem spindexerSubsystem, IntakeSubsystem intakeSubsystem, RobotHardwareMap robot){
        this.spindexerSubsystem = spindexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addCommands(
                new ConditionalCommand(
                        new InstantCommand(),
                        new SequentialCommandGroup(
                                new InstantCommand(() ->  intakeSubsystem.setState(IntakeSubsystem.IntakeState.Idling)),
                                new WaitCommand(200),
                                // 1st Shot
                                new InstantCommand(() -> {
                                    spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.Shooting);
                                    spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.shootingPositions[0]);
                                    spindexerSubsystem.startPositionCheck(0);
                                }),
                                new WaitUntilCommand(spindexerSubsystem::spindexerPositionalReached),
                                new WaitCommand(50),
                                new TransferCMD(robot, spindexerSubsystem, 1),

                                // 2nd Shot
                                new WaitUntilCommand(spindexerSubsystem::spindexerPositionalReached),
                                new TransferCMD(robot, spindexerSubsystem, 2),

                                // 3rd Shot
                                new WaitUntilCommand(spindexerSubsystem::spindexerPositionalReached),
                                new TransferCMD(robot, spindexerSubsystem, 1),

                                // Reset
                                new InstantCommand(() -> {
                                    spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.Intaking);
                                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.Disabled);
                                    spindexerSubsystem.setSpindexerPosition(rConstants.SpindexerConstants.intakingPositions[0]);
                                })
                        ),
                        () -> rConstants.Enums.currentShooterState == rConstants.Enums.ShooterState.Disabled
                )
        );
    }
}