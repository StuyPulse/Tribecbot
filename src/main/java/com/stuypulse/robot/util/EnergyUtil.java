package com.stuypulse.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to get and use energy usage statistics
 * <p>
 * inspired by Mechanical Advantage
 * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/699?u=dandon
 * </p>
 */
public class EnergyUtil {
    private double totalCurrent = 0.0;
    private double totalPowerWatts = 0.0;
    private double totalEnergyWattHours = 0.0;
    private double batteryVoltage = 12.6;
    private double rioCurrent = 0.0;

    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();

    public void logEnergyUsage(String subsystem, double amps) {
        double powerWatts = amps * batteryVoltage; // Supply current draw of the subsystem
        double energyWattHours = joulesToWattHours(powerWatts * Robot.getRobotTime());

        totalCurrent += amps;
        totalPowerWatts += powerWatts;
        totalEnergyWattHours += energyWattHours;

        subsytemCurrents.put(subsystem, amps);
        subsytemPowers.put(subsystem, powerWatts);
        subsytemEnergies.merge(subsystem, energyWattHours, (a, b) -> {
            return a + b;
        });

        // String[] keys = subsystem.split("/|-");
        // if (keys.length < 2) {
        //     return;
        // }

        // String subkey = "";
        // for (int i = 0; i < keys.length - 1; i++) {
        //     subkey += keys[i];
        //     if (i < keys.length - 2) {
        //         subkey += "/";
        //     }
        //     subsytemCurrents.merge(subkey, amps, Double::sum);
        //     subsytemPowers.merge(subkey, powerWatts, Double::sum);
        //     subsytemEnergies.merge(subkey, energyWattHours, Double::sum);
        // }
    }

    public void periodic() {

        SmartDashboard.putNumber("EnergyUtil/Total Supply Current Amps", totalCurrent);
        totalCurrent = 0.0;
        SmartDashboard.putNumber("EnergyUtil/Total Supply Power Watts", totalPowerWatts);
        totalPowerWatts = 0.0;
        SmartDashboard.putNumber("EnergyUtil/Total Used Energy Watt Hours", totalEnergyWattHours);

        for (var entry : subsytemCurrents.entrySet()) {
            SmartDashboard.putNumber("EnergyUtil/Supply Current Amps/" + entry.getKey(), entry.getValue());
            subsytemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemPowers.entrySet()) {
            SmartDashboard.putNumber("EnergyUtil/Power Watts/" + entry.getKey(), entry.getValue());
            subsytemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemEnergies.entrySet()) {
            SmartDashboard.putNumber("EnergyUtil/Energy watt hours/" + entry.getKey(), entry.getValue());
        }
    }

    public double getTotalCurrent() {
        return totalCurrent;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }

    public void setBatteryVoltage(double voltage) {
        this.batteryVoltage = voltage;
    }

    public void setRioCurrent(double current) {
        this.rioCurrent = current;
    }

}
