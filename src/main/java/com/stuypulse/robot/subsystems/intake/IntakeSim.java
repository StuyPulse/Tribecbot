package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class IntakeSim extends Intake {
    private final DCMotorSim intakeMotorSimulated;
    private final PIDController pidController;
    private final ArmFeedforward ffController;

    //Make a new Mechanism 2d
    //Mechanism2d object is your canvas, so the x and y that you input should be representative of the 
    //MechanismRoot2d is your name to find in GLASS (important) and the starting vector (root) of your drawing
    //MechanismLigament2d, this is where you input your starting angle and length (might be more but figure it out)

    //REMEMBER: declare outside, initialize inside.

    //Update the values in periodic (at the end of the webpage) in periodic based on what you know

    //https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    private final Mechanism2d pivotCanvas;
    private final MechanismRoot2d pivotRoot;
    private final MechanismLigament2d ligament;
    
    //need to impl abstract methods
    

    public IntakeSim() {
        //TODO: get actual gear ratio and apply it
        //TODO: potentially add std devs
        intakeMotorSimulated = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 48), DCMotor.getKrakenX60(1));
        pivotCanvas = new Mechanism2d(
            3,
            3
        );  
        pivotRoot = pivotCanvas.getRoot("PivotSim", 2 ,2);
        ligament = pivotRoot.append(new MechanismLigament2d(
            "ligament", 
            2, 
            90
        ));


        pidController = new PIDController(
            1,
            0,
            0
        );
        ffController = new ArmFeedforward(
            0.0,
            0.0,
            0.0,
            0.0,
            0.02
        );

        intakeMotorSimulated.setAngle(Math.toRadians(90));

        

        //TODO: set starting angle NTS (MIGHT NOT BE NEEDED, TEST FIRST)
        

    }

    @Override
    public void periodic() {
        //TODO: change from state to accurate value NTS

        //TODO: check in on Rotation2ds, and change velocity value for ffController if deemed neccessary
        double pidOutput = pidController.calculate(Math.toDegrees(intakeMotorSimulated.getAngularPositionRad()), 180/*getPivotState().getTargetAngle().getDegrees()*/);
        double ffOutput = ffController.calculate(pidController.getSetpoint(), 1);
        
        //pid + ff = voltage
        double voltageInput = pidOutput + ffOutput;
        
        intakeMotorSimulated.setInputVoltage(voltageInput);
        intakeMotorSimulated.update(0.02);

        ligament.setAngle(Math.toDegrees(intakeMotorSimulated.getAngularPositionRad()));
        
    }
    
}