package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Runs the digestion routine 3 times, 0.5sec pause between each state change.
 */
public class IntakeAutoDigest extends SequentialCommandGroup {
    public IntakeAutoDigest() {
        addCommands(
                new IntakeDigest().andThen(new WaitCommand(0.5)).andThen(new IntakeDeploy())
                        .andThen(new WaitCommand(0.5))
                        .andThen(new IntakeDigest().andThen(new WaitCommand(0.5)).andThen(new IntakeDeploy()))
                        .andThen(new WaitCommand(0.5))
                        .andThen(new IntakeDigest().andThen(new WaitCommand(0.5)).andThen(new IntakeDeploy()))
                        .andThen(new WaitCommand(0.5)));
    }
}
