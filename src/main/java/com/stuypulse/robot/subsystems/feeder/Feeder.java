package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private static final Feeder instance;
    private FeederState state;

    static {
        instance = new FeederImpl();
    }

    public static Feeder getInstance() {
        return instance;
    }

    public enum FeederState {
        // STOW(),
        FOWARD(Settings.Feeder.FORWARD_SPEED),
        REVERSE(Settings.Feeder.REVERSE_SPEED),
        STOP(0);
        

        private Number speed;

        private FeederState(Number speed){
            this.speed = speed;
        }

        public double getSpeed(){
            return this.speed.doubleValue();
        }

        private FeederState state;

        protected void Feeder(){
            this.state = FeederState.STOP;
        }

        public FeederState getState(){
            return state;
        }

        public void setState(FeederState state){
            this.state = state;
        }
    }
    

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putString("Feeder/State", state.getState().toString());
        SmartDashboard.putNumber("Feeder/Speed", state.getSpeed());

        
    }
}
