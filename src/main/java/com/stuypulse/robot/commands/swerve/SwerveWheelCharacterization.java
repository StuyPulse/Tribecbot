package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveWheelCharacterization extends Command {

    private static final double ROTATIONAL_RATE = 1.0;
    private static final double DRIVE_RADIUS = Math.sqrt(4.5 * 4.5 + 6.5 * 6.5) * 0.0254;
    private static final double RAMP_RATE = 0.05;

    private final Timer timer = new Timer();
    private final SlewRateLimiter limiter = new SlewRateLimiter(RAMP_RATE);

    private final CommandSwerveDrivetrain swerve;

    private double[] wheelInitial;
    private Rotation2d lastAngle;
    private double gyroDelta;
    private boolean initalReading;

    public SwerveWheelCharacterization() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.restart();
        limiter.reset(0.0);
        gyroDelta = 0.0;
        initalReading = false;
    }

    @Override
    public void execute() {
        double speed = limiter.calculate(ROTATIONAL_RATE);
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(speed)
        );

        if (timer.get() > 1.0) {
            if (!initalReading) {
                wheelInitial = swerve.getRadiusCharacterizationModulePositions();
                lastAngle = swerve.getPigeon2().getRotation2d();
                gyroDelta = 0.0;
                initalReading = true;
            }
        
        Rotation2d currentAngle = swerve.getPigeon2().getRotation2d();
        gyroDelta += Math.abs(currentAngle.minus(lastAngle).getRadians());
        lastAngle = currentAngle;

        double[] wheelCurrent = swerve.getRadiusCharacterizationModulePositions();
        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(wheelCurrent[i] - wheelInitial[i]) / 4.0;
        }

        double wheelRadius = (gyroDelta * DRIVE_RADIUS) / wheelDelta; 

        SmartDashboard.putNumber("Radius Characterization/Radius (m)", wheelRadius);
        SmartDashboard.putNumber("Radius Characterization/Radius (in.)", wheelRadius * 39.3701);
        SmartDashboard.putNumber("Radius Characterization/Gyro Delta", gyroDelta);
        SmartDashboard.putNumber("Radius Characterization/Wheel Delta", wheelDelta);}
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
        .withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        double[] wheelCurrent = swerve.getRadiusCharacterizationModulePositions();
        
        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += (Math.abs(wheelCurrent[i] - wheelInitial[i]) / 4.0);
        }

        double wheelRadius = (gyroDelta * DRIVE_RADIUS) / wheelDelta;

        System.out.println("********** Wheel Radius Characterization Results **********");
        System.out.printf("\tWheel Delta: %.9f radians%n", wheelDelta);
        System.out.printf("\tGyro Delta:  %.9f radians%n", gyroDelta);
        System.out.printf("\tWheel Radius: %.9f meters / %.9f inches%n", wheelRadius, wheelRadius * 39.3701);
    }

    @Override
    public boolean isFinished() {
        return false; // driver cancels manually
    } }