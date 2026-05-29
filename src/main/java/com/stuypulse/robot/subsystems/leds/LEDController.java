/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final static LEDController instance;

    public static boolean isLeftLLDead;
    public static boolean isBackLLDead;
    public static boolean isRightLLDead;
;
    public boolean leftDeadAnimationCleared; 
    public boolean backDeadAnimationCleared; 
    public boolean rightDeadAnimationCleared;

    // private Pose2d lastPoseOnAprilTag;
    // private boolean initialPoseUpdated = false;

    static {
        instance = new LEDController();
    }

    public static LEDController getInstance() {
        return instance;
    }

    private final CANdle leds;
    private CANdleConfiguration candleConfigs;
    private ControlRequest ledPattern = Settings.LED.solidColorRequest.withColor(Settings.LED.DISABLED);

    private LEDController() {
        leftDeadAnimationCleared = false;
        backDeadAnimationCleared = false;
        rightDeadAnimationCleared = false;

        isLeftLLDead = false;
        isBackLLDead = false;
        isRightLLDead = false;

        leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.CANIVORE);
        // lastPoseOnAprilTag = new Pose2d();

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

    // add the flashing aspect based on if we don't see a tag (with debounce or by distance)

    public enum LedState {
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
        RESET(Settings.LED.RESET_HEADING),
        X_WHEELS(Settings.LED.X_WHEELS),
        INTAKE_STOW(Settings.LED.INTAKE_STOW),
        INTAKE_DEPLOYED(Settings.LED.INTAKE_DEPLOYED),
        DISABLED_ALIGNED(Settings.LED.DISABLED_ALIGNED),
        DISABLED(Settings.LED.DISABLED),
        AUTON_COLOR_ONE(Settings.LED.AUTON_ONE),
        AUTON_COLOR_TWO(Settings.LED.AUTON_TWO);

        private RGBWColor color;

        private LedState(RGBWColor color) {
            this.color = color;
        }

        private RGBWColor getColor() {
            return this.color;
        }

        public ControlRequest getAnimation() {
            return Settings.LED.solidColorRequest.withColor(this.color);
        }
    }

    private LedState state = LedState.DISABLED;
    private LedState cachedState = LedState.DISABLED;

    //TODO: make branch for the distance flashing thing
    public void applyPattern() {
        // if (initialPoseUpdated &&
        //         lastPoseOnAprilTag.getTranslation()
        //                 .getDistance(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()) > Settings.LED.APRIL_TAG_DISTANCE_THRESHOLD) {
        //     
        // }

        if (cachedState != state) {
            SolidColor solidColor = (SolidColor) ledPattern;
            solidColor.withColor(state.getColor());

            cachedState = state;
        }
    }

    public void changeState(LedState state) {
        this.state = state;
    }


    public void periodicAfterScheduler() {
        // if (Cameras.LimelightCameras[0].getNumberOfTagsSeen() > 0 ||
        //         Cameras.LimelightCameras[1].getNumberOfTagsSeen() > 0 ||
        //         Cameras.LimelightCameras[2].getNumberOfTagsSeen() > 0) {
        //     lastPoseOnAprilTag = CommandSwerveDrivetrain.getInstance().getPose();
        //     initialPoseUpdated = true;
        // }

        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            applyPattern();
            leds.setControl(ledPattern);
        } else {
            leds.clearAllAnimations();
        }

        //deadAnimationClear booleans ensure we aren't clearing animations 3 times per loop.
        if (isRightLLDead) {
            leds.setControl(Settings.LED.RIGHT_DEAD_STRIP
                    .withColor(Settings.LED.LLDEAD));
            rightDeadAnimationCleared = false;
        } else if (!isRightLLDead && !rightDeadAnimationCleared) {
            leds.clearAllAnimations();
            rightDeadAnimationCleared = true;
        }
        if (isLeftLLDead) {
            leds.setControl(Settings.LED.LEFT_DEAD_STRIP
                    .withColor(Settings.LED.LLDEAD));
            leftDeadAnimationCleared = false;
        } else if (!isLeftLLDead && !leftDeadAnimationCleared) {
            leds.clearAllAnimations();
            leftDeadAnimationCleared = true;
        }
        if (isBackLLDead) {
            leds.setControl(Settings.LED.BACK_DEAD_STRIP
                    .withColor(Settings.LED.LLDEAD));
            backDeadAnimationCleared = false;
        } else if (!isBackLLDead && !backDeadAnimationCleared) {
            leds.clearAllAnimations();
            backDeadAnimationCleared = true;
        }

        DogLog.log("LED/Applied Pattern Name", ledPattern.getName());

        DogLog.log("LED/State Pattern Name", state.getAnimation().getName());
        DogLog.log("LED/Cached State Pattern Name", state.getAnimation().getName());

        DogLog.log("LED/State", state.toString());
        DogLog.log("LED/Cached State", state.toString());
        
        DogLog.log("LED/Is Back LL dead", isBackLLDead);
        DogLog.log("LED/Is Right LL dead", isRightLLDead);
        DogLog.log("LED/Is Left LL dead", isLeftLLDead);

    }
}