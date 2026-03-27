package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootArtefactsCMD extends SequentialCommandGroup {
    private final SpindexerSubsystem spindexerSubsystem;






    public ShootArtefactsCMD(SpindexerSubsystem spindexerSubsystem){
        this.spindexerSubsystem = spindexerSubsystem;

        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(() -> spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.LineUpForCycle)),
                        new WaitUntilCommand(spindexerSubsystem::spindexerPositionReached),
                        new InstantCommand(() -> spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.CyclingArtefacts)),

                        new WaitUntilCommand(spindexerSubsystem::spindexerPositionReached)
                )
        );
    }
}
