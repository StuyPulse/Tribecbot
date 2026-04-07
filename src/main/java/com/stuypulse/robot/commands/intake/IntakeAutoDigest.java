package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAutoDigest extends SequentialCommandGroup {

    public IntakeAutoDigest() {

        addCommands(

            new IntakeDigest().andThen(new WaitCommand(0.25)).andThen(new IntakeDeploy()).andThen(new WaitCommand(0.25))
            .andThen(new IntakeDigest()).andThen(new WaitCommand(0.25)).andThen(new IntakeDeploy()).andThen(new WaitCommand(0.25))

        );

    }
    
}
