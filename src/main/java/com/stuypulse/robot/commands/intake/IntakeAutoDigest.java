package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeAutoDigest extends SequentialCommandGroup {

    public IntakeAutoDigest() {

        addCommands(

            new IntakeDigest().andThen(new IntakeDeploy()).andThen(new IntakeDigest()).andThen(new IntakeDeploy())

        );

    }
    
}
