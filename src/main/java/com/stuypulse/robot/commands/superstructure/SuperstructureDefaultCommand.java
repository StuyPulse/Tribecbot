package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SuperstructureDefaultCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final Superstructure superstructure;
    private boolean trench = false;

    public SuperstructureDefaultCommand() {
        swerve = CommandSwerveDrivetrain.getInstance();
        superstructure = Superstructure.getInstance();

        addRequirements(superstructure);
    }

    @Override
    public void execute() {
        if (swerve.isUnderTrench() && trench == false) {
            CommandScheduler.getInstance().schedule(new SuperstructureStow());
            trench = true;
        } else {
            trench = false;
        }

    }
}
