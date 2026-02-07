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
        canvas = new Mechanism2d(25, 25);
        hopper = canvas.getRoot("Hopper", 8, 10);
        bot = canvas.getRoot("Bot", 8, 5); 
        climber = canvas.getRoot("Climber", 4, 5);

        hopper.append(new MechanismLigament2d(
            "Hopper",
            5,
            0,
            5,
            new Color8Bit(Color.kYellow)
            )
        );
        
        bot.append(new MechanismLigament2d(
            "Bot",
            10,
            0,
            10,
            new Color8Bit(Color.kRed)
        ));

        climber.append(new MechanismLigament2d(
            "Climber",
            2,
            90,
            5,
            new Color8Bit(Color.kOrange)
        ));
    }

    public void update(double height) {
        
        hopper.setPosition(8, height + 5);

        SmartDashboard.putData("Visualizers/ClimberHopper", canvas);
    }
}
    