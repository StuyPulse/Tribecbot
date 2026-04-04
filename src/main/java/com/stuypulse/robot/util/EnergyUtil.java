package com.stuypulse.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.stuypulse.robot.constants.Settings;

import dev.doglog.DogLog;
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
    private double totalCurrentCurrent = 0.0;
    private double totalCurrentPowerWatts = 0.0;
    private double totalEnergyWattHours = 0.0;
    private double batteryVoltage = 12.6;

    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();

    public void logEnergyUsage(String subsystem, double amps) {
        double powerWatts = amps * batteryVoltage; // Supply current draw of the subsystem
        double energyWattHours = powerWatts * Settings.DT;

        totalCurrent += amps;
        totalPowerWatts += powerWatts;
        totalEnergyWattHours += energyWattHours;

        subsytemCurrents.put(subsystem, amps);
        subsytemPowers.put(subsystem, powerWatts);
        subsytemEnergies.merge(subsystem, energyWattHours, Double::sum);

        String[] keys = subsystem.split("/|-");
        if (keys.length < 2) {
            return;
        }

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsytemCurrents.merge(subkey, amps, Double::sum);
            subsytemPowers.merge(subkey, powerWatts, Double::sum);
            subsytemEnergies.merge(subkey, energyWattHours, Double::sum);
        }
    }

    public void periodic() {
        if (Settings.DEBUG_MODE.get()) {
            // energy used over the total time the robot has been on
            SmartDashboard.putNumber("EnergyUtil/Total used Supply Current draw Amps", totalCurrent);
            SmartDashboard.putNumber("EnergyUtil/total used Power Watts", totalPowerWatts);
            SmartDashboard.putNumber("EnergyUtil/Total Used Energy Watt Hours", joulesToWattHours(totalEnergyWattHours));

            // energy used over the current command schedule loop 
            totalCurrentCurrent = totalCurrent - totalCurrentCurrent;
            totalCurrentPowerWatts = totalPowerWatts - totalCurrentPowerWatts;
            SmartDashboard.putNumber("EnergyUtil/Total current Current", totalCurrentCurrent);
            SmartDashboard.putNumber("EnergyUtil/Total current Power Watts", totalCurrentCurrent);

            for (var entry : subsytemCurrents.entrySet()) {
                SmartDashboard.putNumber("EnergyUtil/Supply Current Amps/" + entry.getKey(), entry.getValue());
                subsytemCurrents.put(entry.getKey(), 0.0);
            }
            for (var entry : subsytemPowers.entrySet()) {
                SmartDashboard.putNumber("EnergyUtil/Power Watts/" + entry.getKey(), entry.getValue());
                subsytemPowers.put(entry.getKey(), 0.0);
            }
            for (var entry : subsytemEnergies.entrySet()) {
                SmartDashboard.putNumber("EnergyUtil/Energy Watt Hours/" + entry.getKey(), joulesToWattHours(entry.getValue()));
            }
        }
        else {
            // energy used over the total time the robot has been on
            DogLog.log("EnergyUtil/Total used Supply Current draw Amps", totalCurrent);
            DogLog.log("EnergyUtil/total used Power Watts", totalPowerWatts);
            DogLog.log("EnergyUtil/Total Used Energy Watt Hours", joulesToWattHours(totalEnergyWattHours));

            // energy used over the current command schedule loop 
            totalCurrentCurrent = totalCurrent - totalCurrentCurrent;
            totalCurrentPowerWatts = totalPowerWatts - totalCurrentPowerWatts;
            DogLog.log("EnergyUtil/Total current Current", totalCurrentCurrent);
            DogLog.log("EnergyUtil/Total current Power Watts", totalCurrentCurrent);

            for (var entry : subsytemCurrents.entrySet()) {
                DogLog.log("EnergyUtil/Supply Current Amps/" + entry.getKey(), entry.getValue());
                subsytemCurrents.put(entry.getKey(), 0.0);
            }
            for (var entry : subsytemPowers.entrySet()) {
                DogLog.log("EnergyUtil/Power Watts/" + entry.getKey(), entry.getValue());
                subsytemPowers.put(entry.getKey(), 0.0);
            }
            for (var entry : subsytemEnergies.entrySet()) {
                DogLog.log("EnergyUtil/Energy Watt Hours/" + entry.getKey(), joulesToWattHours(entry.getValue()));
            }
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

}
