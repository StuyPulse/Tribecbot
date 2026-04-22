/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.DriverConstants.Driver.Drive;
import com.stuypulse.robot.constants.DriverConstants.Driver.Turn;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.intake.IntakeSetDigesting;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SwerveDriveSOTM extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Superstructure superstructure;

    private final Gamepad driver;

    private final VStream speed;
    private final IStream turn;

    private final BStream isIdle;
    private boolean isIdleInit;

    public SwerveDriveSOTM(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        superstructure = Superstructure.getInstance();

        speed = VStream.create(this::getDriverInputAsVelocity)
                .filtered(
                        new VDeadZone(Drive.DEADBAND),
                        x -> x.clamp(1),
                        x -> x.pow(Drive.POWER),
                        x -> x.mul(Swerve.Constraints.MAX_VELOCITY_SOTM_M_PER_S),
                        new VRateLimit(Settings.Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED_SOTM),
                        new VLowPassFilter(Drive.RC));

        turn = IStream.create(driver::getRightX)
                .filtered(
                        x -> SLMath.deadband(x, Turn.DEADBAND),
                        x -> SLMath.spow(x, Turn.POWER),
                        x -> x * Swerve.Constraints.MAX_ANGULAR_VEL_SOTM_RAD_PER_S,
                        new LowPassFilter(Turn.RC));

        isIdle = BStream.create(
                () -> getDriverInputAsVelocity().magnitude() <= Drive.DEADBAND
                        && Math.abs(driver.getRightX()) <= Turn.DEADBAND)
                .filtered(new BDebounce.Rising(0.1), new BDebounce.Falling(0.1));

        this.driver = driver;
        isIdleInit = false;

        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    @Override
    public void initialize() {
        isIdleInit = false;
    }

    @Override
    public void execute() {
        DogLog.log("Swerve/SOTM/Idle?", isIdle.get());

        if (isIdle.get()) {
            swerve.setControl(new SwerveRequest.SwerveDriveBrake());
            // this runs the intake somewhat violently
            if (!isIdleInit) {
                CommandScheduler.getInstance()
                        .schedule(new IntakeAutoDigest().repeatedly().onlyWhile(() -> isIdle.get())
                                .andThen(new IntakeDeploy()).finallyDo(() -> Intake.getInstance().setIntakeDigesting(false)));
            }
            isIdleInit = true;
        } else {
            Vector2D velocity = speed.get();
            swerve.setControl(swerve.getFieldCentricSwerveRequest()
                    .withVelocityX(velocity.x)
                    .withVelocityY(velocity.y)
                    .withRotationalRate(-turn.get()));
            isIdleInit = false;
        }

    }

    @Override
    public boolean isFinished() {
        SuperstructureState state = superstructure.getState();
        if (state == SuperstructureState.SOTM) {
            return false;
        } else {
            return true;
        }
    }
}
