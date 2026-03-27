package com.stuypulse.robot.commands.vision;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class EnablePortForwarding extends InstantCommand {
    private String hostname;

    public EnablePortForwarding(String hostname) {
        this.hostname = hostname;
    }

    @Override
    public void initialize() {
        PortForwarder.remove(5801);
        PortForwarder.add(5801, hostname, 5801);
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
