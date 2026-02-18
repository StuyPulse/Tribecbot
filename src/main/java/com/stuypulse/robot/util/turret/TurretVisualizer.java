package com.stuypulse.robot.util.turret;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class TurretVisualizer {
    public static TurretVisualizer instance;

    static {
        instance = new TurretVisualizer();
    }

    public static TurretVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;
    private double width, height;

    private final MechanismRoot2d turretPivot;
    private final MechanismLigament2d turret;

    private TurretVisualizer() {
        width = Units.inchesToMeters(20);
        height = Units.inchesToMeters(20);

        canvas = new Mechanism2d(width, height);

        turretPivot = canvas.getRoot("Turret Pivot", width/2, height/2);

        turret = new MechanismLigament2d(
            "Turret",
            Units.inchesToMeters(5),
            0.0,
            4,
            new Color8Bit(Color.kBlueViolet)
        );

        turretPivot.append(turret);
    }

    public void updateTurretAngle(Rotation2d turretAngle, boolean atTargetAngle) {
        turretPivot.setPosition(width/2, height/2);
        turret.setAngle(turretAngle);

        turret.setColor(atTargetAngle ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));

        SmartDashboard.putData("Visualizers/Turret", canvas);
    }
}