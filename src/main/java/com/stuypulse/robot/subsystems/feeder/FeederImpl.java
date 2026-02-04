package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class FeederImpl extends Feeder {

    private final TalonFX motor1;
    private final TalonFX motor2;

    public FeederImpl() {
        super();
         motor1 = new TalonFX(Ports.Feeder.FEEDER1);
         Motors.Feeder.motorConfig.configure(motor1);

         motor2 = new TalonFX(Ports.Feeder.FEEDER2);
         Motors.Feeder.motorConfig.configure(motor2);
    }

   @Override
   public void periodic(){
    super.periodic();

    if(Settings.EnabledSubsystems.FEEDER.get()){
        motor1.set(motor1.get());
        motor2.set(motor2.get());
    }
   }
}

