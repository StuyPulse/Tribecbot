/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final static LEDController instance;

    public static boolean isLeftLLDead = false;
    public static boolean isBackLLDead = false;
    public static boolean isRightLLDead = false;

    private Pose2d lastPoseOnAprilTag;
    private boolean initialPoseUpdated = false;

    static {
        instance = new LEDController();
    }

    public static LEDController getInstance() {
        return instance;
    }

    private final CANdle leds;
    private CANdleConfiguration candleConfigs;
    private ControlRequest ledPattern = Settings.LED.solidColorRequest.withColor(Settings.LED.DISABLED);

    // different portions of the LED should be a different color to indicate whether
    // certain limelights are dead
    // add the flashing aspect based on if we don't see a tag (with debounce)
    // one way to go further with the flashing aspect is make it flash faster over
    // DISTANCE (since last tag was seen) rather than time

    public enum LEDSTATE {
        PASSING_TRENCH(Settings.LED.PASSING_TRENCH),
        IS_BEHIND_HUB(Settings.LED.IS_BEHIND_HUB),
        TURRET_WRAPPING(Settings.LED.TURRET_WRAPPING),
        SHOOT_IN_PLACE(Settings.LED.SHOOT_IN_PLACE),
        SOTM_ON(Settings.LED.SOTM_ON),
        FOTM_ON(Settings.LED.FOTM_ON),
        LEFT_CORNER(Settings.LED.LEFT_CORNER),
        RIGHT_CORNER(Settings.LED.RIGHT_CORNER),
        KB_DISTANCE(Settings.LED.KB_DISTANCE),
        STOP_ROLLERS(Settings.LED.STOP_ROLLERS),
        RESET_HEADING(Settings.LED.RESET_HEADING),
        X_WHEELS(Settings.LED.X_WHEELS),
        INTAKE_STOW(Settings.LED.INTAKE_STOW),
        INTAKE_DEPLOYED(Settings.LED.INTAKE_DEPLOYED),
        DISABLED_ALIGNED(Settings.LED.DISABLED_ALIGNED),
        DISABLED(Settings.LED.DISABLED);

        private RGBWColor color;

        private LEDSTATE(RGBWColor color) {
            this.color = color;
        }

        private RGBWColor getColor() {
            return this.color;
        }

        public ControlRequest getAnimation() {
            return Settings.LED.solidColorRequest.withColor(this.color);
        }
    }

    private LEDSTATE state = LEDSTATE.DISABLED;
    private LEDSTATE cachedState = LEDSTATE.DISABLED;

    // CHANGE apply pattern command to change state

    public void applyPattern() {
        if (cachedState != state) {
            // if (cachedState.getAnimation().getName() != "SolidColor") {
            //     leds.clearAllAnimations();
            // } 
              // clearAllAnimations every loop
            // if (!(cachedState.getAnimation().getName().equals(state.getAnimation().getName()))) {
            //     this.ledPattern = state.getAnimation();
            // }

            // else if (ledPattern instanceof SolidColor) {
                SolidColor solidColor = (SolidColor) ledPattern;
                solidColor.withColor(state.getColor());
                // SolidColor.class.cast(ledPattern).withColor(null); //change if neccesary
            // }
            cachedState = state;
        }
    }

    public void changeState(LEDSTATE state) {
        this.state = state;
    }

    private LEDController() {
        leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.CANIVORE);
        lastPoseOnAprilTag = new Pose2d();

        candleConfigs = new CANdleConfiguration()
                .withLED(
                        new LEDConfigs()
                                .withBrightnessScalar(1.0)
                                .withStripType(StripTypeValue.GRB)
                                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning))

                .withCANdleFeatures(
                        new CANdleFeaturesConfigs().withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled));

        leds.getConfigurator().apply(candleConfigs);

        leds.setControl(ledPattern);
    }

    public void periodicAfterScheduler() {
        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            applyPattern();
            leds.setControl(ledPattern);
        } else {
            leds.clearAllAnimations();
        }

        // reflective of the 3 LED gap between them
        if (isRightLLDead) {
            //TODO: when it goes back on CLEAR ANIMATIONS !!
            leds.setControl(new SolidColor(Settings.LED.LED_LENGTH - 4, Settings.LED.LED_LENGTH - 1)
                    .withColor(Settings.LED.RIGHTDEAD));
        }
        if (isLeftLLDead) {
            leds.setControl(new SolidColor(Settings.LED.LED_LENGTH - 11, Settings.LED.LED_LENGTH - 8)
                    .withColor(Settings.LED.LEFTDEAD));
        }
        if (isBackLLDead) {
            leds.setControl(new SolidColor(Settings.LED.LED_LENGTH - 18, Settings.LED.LED_LENGTH - 15)
                    .withColor(Settings.LED.BACKDEAD));
        }

        if (Cameras.LimelightCameras[0].getNumberOfTagsSeen() > 0 ||
                Cameras.LimelightCameras[1].getNumberOfTagsSeen() > 0 ||
                Cameras.LimelightCameras[2].getNumberOfTagsSeen() > 0) {
            lastPoseOnAprilTag = CommandSwerveDrivetrain.getInstance().getPose();
            initialPoseUpdated = true;
        }

        if (initialPoseUpdated &&
                lastPoseOnAprilTag.getTranslation()
                        .getDistance(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()) > Settings.LED.APRIL_TAG_DISTANCE_THRESHOLD) {
            //TODO: add flashing
        }

        DogLog.log("LED/Applied Pattern Name", ledPattern.getName());

        DogLog.log("LED/State Pattern Name", state.getAnimation().getName());
        DogLog.log("LED/Cached State Pattern Name", state.getAnimation().getName());

        DogLog.log("LED/State", state.toString());
        DogLog.log("LED/Cached State", state.toString());

    }
}