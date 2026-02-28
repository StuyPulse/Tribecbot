package com.stuypulse.robot.util;

import java.util.concurrent.TransferQueue;

import com.fasterxml.jackson.annotation.JsonFormat.Feature;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FMSutil {
    private Timer timer = new Timer();
    private FieldState fieldState;
    private double timeLeft;
    boolean auto;
    private FieldState[] fieldStates = {fieldState.TRANSITION, fieldState.SHIFT1, fieldState.SHIFT2,
            fieldState.SHIFT3, fieldState.SHIFT4, fieldState.ENDGAME };

    public static enum FieldState {
        AUTO(0.0, 20.0),
        TRANSITION(0.0, 10),
        SHIFT1(10.0, 35.0),
        SHIFT2(35.0, 60.0),
        SHIFT3(60.0, 85.0),
        SHIFT4(85.0, 110.0),
        ENDGAME(110.0, 140.0);

        // public final do.0uble[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
        // public final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
        private double startT;
        private double endT;

        private FieldState(double startT, double endT) {
            this.startT = startT;
            this.endT = endT;
        }

        public boolean isActive(double time) {
            if (startT <= time && time < endT) {
                return true;
            } else {
                return false;
            }
        }

        public double timeLeft(double time) {
            return endT - time;
        }

        public double timeElapsed(double time) {
            return time - startT;
        }
    };


    public FMSutil(boolean auto) {
        timer = new Timer();
        timer.start();
        this.auto = auto;
    }

    public FMSutil() {
        this(false);
    }

    public void resetTimer() {
        timer.reset();
    }

    public FieldState getFieldState() {
        FieldState activeState = FieldState.AUTO;
        if (auto) return FieldState.AUTO;
        for (FieldState state : fieldStates) {
            if (state.isActive(timer.get())) { activeState = state; break;}
        }
        return activeState;
    }
    
    public boolean isActiveShift() {
        boolean wonAuto = didWinAuto();
        switch (getFieldState()) {
            case AUTO: 
                return true;
            case TRANSITION: 
                return false;
            case ENDGAME:
                return true;
            case SHIFT1:
                return (wonAuto) ? true : false;
            case SHIFT2: 
                return (wonAuto) ? false : true;
            case SHIFT3:
                return (wonAuto) ? true : false;
            case SHIFT4:
                return (wonAuto) ? false : false;
            default:
                return false;
        }
    }

    public boolean didWinAuto() {
        String winner = DriverStation.getGameSpecificMessage(); 
        String currentAlliance = (DriverStation.getAlliance().get() == Alliance.Blue) ? "B" : "R";      
        if (winner.isEmpty()) {
            DriverStation.reportWarning("Arena Fault, no alliance won data", true);
            SmartDashboard.putBoolean("FMSUtil/nodata?", true);
            return true; // Assume we won :)
        } else if (currentAlliance.equalsIgnoreCase(winner)) {
            return true;
        } else {
            return false;
        }
    }

    public double getTimeLeftInShift() {
        return getFieldState().endT - timer.get();
    }

    
}
