package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.commands.leds.LEDApplyState;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.LEDController.LedState;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Runs the digestion routine 3 times, 0.5sec pause between each state change.
 */
public class IntakeAutoDigest extends SequentialCommandGroup {
    public IntakeAutoDigest() {
        addRequirements(LEDController.getInstance());
        addCommands(
                new LEDApplyState(LedState.INTAKE_DIGEST),
                new IntakeDigest().andThen(new WaitCommand(0.5)).andThen(new IntakeDeploy())
                        .andThen(new WaitCommand(0.5))
                        .andThen(new IntakeDigest().andThen(new WaitCommand(0.5)).andThen(new IntakeDeploy()))
                        .andThen(new WaitCommand(0.5))
                        .andThen(new IntakeDigest().andThen(new WaitCommand(0.5)).andThen(new IntakeDeploy()))
                        .andThen(new WaitCommand(0.5)));
                CommandScheduler.getInstance().schedule(new LEDDefaultCommand());
    }
}
