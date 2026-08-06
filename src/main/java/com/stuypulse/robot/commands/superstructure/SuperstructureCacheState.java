package com.stuypulse.robot.commands.superstructure;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.DriverConstants.Driver.Drive;
import com.stuypulse.robot.constants.DriverConstants.Driver.Turn;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj2.command.Command;


public class SuperstructureCacheState extends Command {
    SuperstructureState cachedState;
    Superstructure superstructure;
    CommandSwerveDrivetrain swerve;
    Gamepad driver;
    BStream isIdle;

    public SuperstructureCacheState(Gamepad driver) {
        superstructure = Superstructure.getInstance();
        swerve = CommandSwerveDrivetrain.getInstance();
        this.driver = driver;

        isIdle = BStream.create(
                () -> getDriverInputAsVelocity().magnitude() <= Drive.DEADBAND && Math.abs(driver.getRightX()) <= Turn.DEADBAND)
                .filtered(new BDebounce.Both(0.1));

        this.cachedState = superstructure.getState();

        addRequirements(superstructure, swerve);
   } 

    @Override
    public void initialize() {
        this.cachedState = superstructure.getState();
        superstructure.setState(SuperstructureState.INTERPOLATION);
        SwerveRequest request = new SwerveRequest.SwerveDriveBrake();
        swerve.setControl(request);
    }

    @Override
    public boolean isFinished() {
        return !isIdle.get();
    }

    @Override
    public void end(boolean interrupted) {
        // this presumes that you already overrode the state in the following part of the command chain
        superstructure.setState(cachedState);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }
}
