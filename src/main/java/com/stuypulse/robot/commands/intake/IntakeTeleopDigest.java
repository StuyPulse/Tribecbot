package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeTeleopDigest extends SequentialCommandGroup {

    public IntakeTeleopDigest() {

        addCommands(

            new IntakeDigest().andThen(new WaitCommand(0.5)).andThen(new IntakeDeploy()).andThen(new WaitCommand(0.5))

        );

    }
    
}
