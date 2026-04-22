/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import dev.doglog.DogLog;

public class FMSUtil {

    private final Timer timer = new Timer();
    private boolean autoMode;
    private boolean autoOverride = false;

    public enum FieldState {
        AUTO(0.0, 20.0),
        TRANSITION(0.0, 10.0),
        SHIFT_1(10.0, 35.0),
        SHIFT_2(35.0, 60.0),
        SHIFT_3(60.0, 85.0),
        SHIFT_4(85.0, 110.0),
        ENDGAME(110.0, 140.0);

        private final double startTime;
        private final double endTime;

        FieldState(double startTime, double endTime) {
            this.startTime = startTime;
            this.endTime = endTime;
        }

        public boolean isActive(double time) {
            return startTime <= time && time < endTime;
        }

        public double timeLeft(double time) {
            if (this == FieldState.SHIFT_4) {
                return Math.max(0.0, 140.0 - time);
            } else {
                return Math.max(0.0, endTime - time);
            }
        }

        public double timeElapsed(double time) {
            return Math.max(0.0, time - startTime);
        }
    }
    
    public FMSUtil(boolean autoMode) {
        this.autoMode = autoMode;
        timer.start();
    }

    public FMSUtil() {
        this(true);
    }

    public void restartTimer(boolean autoMode) {
        timer.restart();
        this.autoMode = autoMode;
    }

    public FieldState getCurrentFieldState() {
        if (autoMode) {
            return FieldState.AUTO;
        }

        double time = timer.get();

        for (FieldState state : FieldState.values()) {
            if (state != FieldState.AUTO && state.isActive(time)) {
                return state;
            }
        }

        return FieldState.ENDGAME;
    }

    public boolean isActiveShift() {
        boolean wonAuto = didWinAuto();

        switch (getCurrentFieldState()) {
            case AUTO:
                return true;
            case TRANSITION:
                return true;
            case ENDGAME:
                return true;
            case SHIFT_1:
                return wonAuto;
            case SHIFT_2:
                return !wonAuto;
            case SHIFT_3:
                return wonAuto;
            case SHIFT_4:
                return !wonAuto;
            default:
                return false;
        }
    }

    public void overrideFMSAutoVictor(boolean didWin) {
        this.autoOverride = didWin;
    }

    public boolean didWinAuto() {
        String winner = DriverStation.getGameSpecificMessage();
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();

        if (winner == null || winner.isEmpty() || allianceOpt.isEmpty()) {
            DriverStation.reportWarning("No FMS auto winner data available", false);
            DogLog.log("FMSUtil/No Auto Winner Data", true);
            return autoOverride;
        }

        String allianceLetter = allianceOpt.get() == Alliance.Blue ? "B" : "R";

        return allianceLetter.equalsIgnoreCase(winner);
    }

    public double getTimeLeftInShift() {
        return getCurrentFieldState().timeLeft(timer.get());
    }
}