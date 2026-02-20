/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climberhopper;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimberHopperVisualizer {
    private static final ClimberHopperVisualizer instance;

    static {
        instance = new ClimberHopperVisualizer();
    }

    public static ClimberHopperVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;
    private final MechanismRoot2d hopper;
    private final MechanismRoot2d bot;
    private final MechanismRoot2d climber;

    ClimberHopperVisualizer() {
        canvas = new Mechanism2d(2.5, 2.5);
        hopper = canvas.getRoot("Hopper", 0.8, 1);
        bot = canvas.getRoot("Bot", 0.8, 0.5); 
        climber = canvas.getRoot("Climber", 0.4, 0.5);

        hopper.append(new MechanismLigament2d(
            "Hopper",
            0.5,
            0,
            0.5,
            new Color8Bit(Color.kYellow)
            )
        );
        
        bot.append(new MechanismLigament2d(
            "Bot",
            1,
            0,
            1,
            new Color8Bit(Color.kRed)
        ));

        climber.append(new MechanismLigament2d(
            "Climber",
            0.2,
            90,
            0.5,
            new Color8Bit(Color.kOrange)
        ));
    }

    public void update(double height) {

        hopper.setPosition(0.8, height + 0.5);

        SmartDashboard.putData("Visualizers/ClimberHopper", canvas);
    }
}
