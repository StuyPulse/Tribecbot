// package com.stuypulse.robot.commands.vision;

// import edu.wpi.first.net.PortForwarder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.InstantCommand;

// public class EnablePortForwarding extends InstantCommand {
//     private String hostname;

//     public EnablePortForwarding(String hostname) {
//         this.hostname = hostname;
//     }

//     @Override
//     public void initialize() {
//         PortForwarder.remove(10000);
//         PortForwarder.add(10000, hostname, 5801);
//         SmartDashboard.putString("Limelight/Limelight Being Port Forwarded", hostname);
//     }
    
//     @Override
//     public boolean runsWhenDisabled() {
//         return true;
//     }
// }
