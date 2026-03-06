package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveWheelRadiusCharacterization extends Command {

    /*
     * - Triple check the track width and length
     * - Double check the gear ratio in getWheelDrivePositionsRadians()
     * - Allow the robot to rotate 3-5 times when running the routine
    */

    private static final double HALF_TRACK_WIDTH_INCHES = 9.0;
    private static final double HALF_TRACK_LENGTH_INCHES = 13.0;

    private static final double DRIVE_RADIUS_INCHES = Math.sqrt(HALF_TRACK_WIDTH_INCHES * HALF_TRACK_WIDTH_INCHES + HALF_TRACK_LENGTH_INCHES * HALF_TRACK_LENGTH_INCHES);
    private static final double DRIVE_RADIUS_METERS = Units.inchesToMeters(DRIVE_RADIUS_INCHES);

    private static final double TARGET_ROTATIONAL_RATE = 1.5; // rad/s about center of rotation
    private static final double RAMP_TIME = 0.75; // seconds to reach target

    private final Timer timer = new Timer();

    private final CommandSwerveDrivetrain swerve;

    private double[] wheelInitialRad;
    private Rotation2d lastAngle;
    private double gyroDeltaRad;
    private boolean initalReading;

    public SwerveWheelRadiusCharacterization() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.restart();
        // limiter.reset(0.0);
        gyroDeltaRad = 0.0;
        initalReading = false;
    }

    @Override
    public void execute() {
        double elapsed = timer.get();

        double commandedRate;

        if (elapsed < RAMP_TIME) {
            commandedRate = TARGET_ROTATIONAL_RATE * (elapsed / RAMP_TIME); // Ramping omega proportional to RAMP_TIME
        } else {
            commandedRate = TARGET_ROTATIONAL_RATE; // Steady state omega
        }

        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(commandedRate)
        );

        // Wait half a second after reaching target omega to start data collection
        if (timer.get() > RAMP_TIME + 0.5) {
            if (!initalReading) {
                wheelInitialRad = swerve.getWheelDrivePositionsRadians();
                lastAngle = swerve.getPigeon2().getRotation2d();
                gyroDeltaRad = 0.0;
                initalReading = true;
            }
        
            Rotation2d currentAngle = swerve.getPigeon2().getRotation2d();
            gyroDeltaRad += Math.abs(currentAngle.minus(lastAngle).getRadians());
            lastAngle = currentAngle;

            double[] wheelCurrentRad = swerve.getWheelDrivePositionsRadians();
            double wheelDeltaRad = 0.0;
            for (int i = 0; i < 4; i++) {
                wheelDeltaRad += Math.abs(wheelCurrentRad[i] - wheelInitialRad[i]) / 4.0;
            }

            double wheelRadius = (gyroDeltaRad * DRIVE_RADIUS_METERS) / wheelDeltaRad; 

            SmartDashboard.putNumber("Radius Characterization/Radius (m)", wheelRadius);
            SmartDashboard.putNumber("Radius Characterization/Radius (in.)", Units.metersToInches(wheelRadius));
            SmartDashboard.putNumber("Radius Characterization/Gyro Delta (rad)", gyroDeltaRad);
            SmartDashboard.putNumber("Radius Characterization/Wheel Delta (rad)", wheelDeltaRad);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));

        double[] wheelCurrentRad = swerve.getWheelDrivePositionsRadians();
        
        double wheelDeltaRad = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDeltaRad += (Math.abs(wheelCurrentRad[i] - wheelInitialRad[i]) / 4.0);
        }

        double wheelRadiusMeters = (gyroDeltaRad * DRIVE_RADIUS_METERS) / wheelDeltaRad;
        double wheelRadiusInches = Units.metersToInches(wheelRadiusMeters);

        System.out.println("********** Wheel Radius Characterization Results **********");
        System.out.printf("\tWheel Delta: %.9f radians%n", wheelDeltaRad);
        System.out.printf("\tGyro Delta:  %.9f radians%n", gyroDeltaRad);
        System.out.printf("\tWheel Radius: %.9f meters / %.9f inches%n", wheelRadiusMeters, wheelRadiusInches);
    }

    @Override
    public boolean isFinished() {
        return false; // driver cancels manually
    } }