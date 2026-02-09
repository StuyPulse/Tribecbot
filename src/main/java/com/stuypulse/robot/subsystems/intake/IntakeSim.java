package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSim extends Intake {
    private final DCMotorSim intakeSimMotor;
    private PIDController pidController;
    private ArmFeedforward ffController;

    Mechanism2d pivotCanvas;
    MechanismRoot2d pivotRoot;
    MechanismLigament2d pivotLigament;

    public IntakeSim() {
        //TODO: potentially add std devs
        intakeSimMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                Settings.Intake.JKgMetersSquared,
                Settings.Intake.GEAR_RATIO
            ), 
            DCMotor.getKrakenX60(1));

        pivotCanvas = new Mechanism2d(   
            4,
            4
        );  

        pivotRoot = pivotCanvas.getRoot(
            "Pivot_Root",
            2 ,
            2);
        pivotLigament = pivotRoot.append(new MechanismLigament2d(
            "Pivot_Ligament", 
            2, 
            90
        ));

        pidController = new PIDController( 
            Gains.Intake.Pivot.kP,
            Gains.Intake.Pivot.kI,
            Gains.Intake.Pivot.kD
        );
        ffController = new ArmFeedforward(
            Gains.Intake.Pivot.kS,
            Gains.Intake.Pivot.kG,
            Gains.Intake.Pivot.kV,
            Gains.Intake.Pivot.kA,
            Settings.Intake.dT
        );

        intakeSimMotor.setAngle(Math.toRadians(90));
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(getCurrentAngle().getRotations() - getIntakeState().getTargetAngle().getRotations()) < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations(); 
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(SLMath.clamp(
            Math.toDegrees(intakeSimMotor.getAngularPositionRad()),
            Settings.Intake.PIVOT_MIN_ANGLE.getDegrees(),
            Settings.Intake.PIVOT_MAX_ANGLE.getDegrees()
        ));
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Pivot_Mechanism2d_SIM", pivotCanvas);
        //TODO: change velocity value for ffController (?)
        double pidOutput = pidController.calculate(getCurrentAngle().getDegrees(), getIntakeState().getTargetAngle().getDegrees());
        double ffOutput = ffController.calculate(pidController.getSetpoint(), 1);
        
        double voltageInput = SLMath.clamp(
            pidOutput + ffOutput,
            Settings.Intake.VOLTAGE_MIN,
            Settings.Intake.VOLTAGE_MAX
        );
    
        intakeSimMotor.setInputVoltage(voltageInput);
        intakeSimMotor.update(Settings.Intake.dT);

        pivotLigament.setAngle(getCurrentAngle().getDegrees());

        SmartDashboard.putNumber("INTAKE_SIMULATION/ Voltage", voltageInput);
        SmartDashboard.putNumber("INTAKE_SIMULATION/ Actual Angle (DEG)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("INTAKE_SIMULATION/ Target Angle (DEG)", getIntakeState().getTargetAngle().getDegrees());
        SmartDashboard.putString("INTAKE_SIMULATION/ Target State", getIntakeState().toString());
        SmartDashboard.putNumber("INTAKE_SIMULATION/ PID output", pidOutput);
        SmartDashboard.putNumber("INTAKE_SIMULATION/ FF output", pidOutput);
        SmartDashboard.putNumber("INTAKE_SIMULATION/ PID Setpoint", pidController.getSetpoint());
        SmartDashboard.putBoolean("INTAKE_SIMULATION/ at Target Angle?", isAtTargetAngle());  
        SmartDashboard.putNumber("Intake/Settings/Pivot/MAX ANGLE", Settings.Intake.PIVOT_MAX_ANGLE.getDegrees());
        SmartDashboard.putNumber("Intake/Settings/Pivot/MIN ANGLE", Settings.Intake.PIVOT_MIN_ANGLE.getDegrees());
    }
}