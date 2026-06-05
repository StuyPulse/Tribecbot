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
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final static LEDController instance;

    public static boolean isLeftLLDead;
    public static boolean isBackLLDead;
    public static boolean isRightLLDead;;

    private Pose2d lastPoseOnAprilTag;
    private boolean initialPoseUpdated;

    private boolean needToBeFade;
    private boolean usingFade;

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
        isLeftLLDead = false;
        isBackLLDead = false;
        isRightLLDead = false;

        initialPoseUpdated = false;
        needToBeFade = false;
        usingFade = false;

        lastPoseOnAprilTag = new Pose2d();

        leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.CANIVORE);

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

    // add the flashing aspect based on if we don't see a tag (with debounce or by
    // distance)

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

    // TODO: make branch for the distance flashing thing

    // refactor to kow how many are dead.
    // Then knowing how many are dead, you clear animations,
    // then normal between start to start of first dead strip, then end to first
    public void applyPattern() {
        if (initialPoseUpdated &&
                lastPoseOnAprilTag.getTranslation()
                        .getDistance(CommandSwerveDrivetrain.getInstance().getPose()
                                .getTranslation()) > Settings.LED.APRIL_TAG_DISTANCE_THRESHOLD
                && !DriverStation.isAutonomous()) {
            needToBeFade = true;
        } else {
            needToBeFade = false;
        }

        // TODO: potentially make this go both ways - if need be
        boolean shouldClear = (ledPattern instanceof SingleFadeAnimation && !needToBeFade
                || ledPattern instanceof SolidColor && needToBeFade)
                        ? true
                        : false;

        if (cachedState != state || shouldClear) {
            // what actually changes led pattern to fade or whatever
            ledPattern = needToBeFade ? Settings.LED.singleFadeAnimation : Settings.LED.solidColorRequest;

            // this what clears our animations once (at most) per loop
            if (shouldClear)
                leds.clearAllAnimations();

            // used to change color because control requests dont all have different colors
            if (ledPattern instanceof SolidColor) {
                Settings.LED.solidColorRequest.withColor(state.getColor());
                usingFade = false;
            } else if (ledPattern instanceof SingleFadeAnimation) {
                Settings.LED.singleFadeAnimation.withColor(state.getColor());
                usingFade = true;
            }

            cachedState = state;
        }
    }

    public void changeState(LedState state) {
        this.state = state;
    }

    public void periodicAfterScheduler() {
        if (Cameras.LimelightCameras[0].getNumberOfTagsSeen() > 0 ||
                Cameras.LimelightCameras[1].getNumberOfTagsSeen() > 0 ||
                Cameras.LimelightCameras[2].getNumberOfTagsSeen() > 0) {
            lastPoseOnAprilTag = CommandSwerveDrivetrain.getInstance().getPose();
            initialPoseUpdated = true;
        }

        DogLog.log("LED/last pose updated", this.lastPoseOnAprilTag);
        DogLog.log("LED/distance from last pose", lastPoseOnAprilTag.getTranslation()
                .getDistance(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()));

        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            applyPattern();
            // led.setControl(ledPattern);
        } else {
            leds.clearAllAnimations();
        }

        // EXTRA STRIP TO THE SIDE
        if (usingFade) {
            leds.setControl(
                    Settings.LED.singleFadeAnimation
                            .withLEDStartIndex(Settings.LED.LEFT_DEAD_STRIP.LEDStartIndex - 1)
                            .withLEDEndIndex(Settings.LED.LEFT_DEAD_STRIP.LEDStartIndex - 1));
        } else {
            leds.setControl(
                    Settings.LED.solidColorRequest
                            .withLEDStartIndex(Settings.LED.LEFT_DEAD_STRIP.LEDStartIndex - 1)
                            .withLEDEndIndex(Settings.LED.LEFT_DEAD_STRIP.LEDStartIndex - 1));
        }

        // RIGHT LL
        if (isRightLLDead) {

            leds.setControl(Settings.LED.RIGHT_DEAD_STRIP
                    .withColor(Settings.LED.LLDEAD));
            if (usingFade) {
                leds.setControl(
                        Settings.LED.singleFadeAnimation
                                .withLEDStartIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDEndIndex + 1) // 28 (MAX)
                                .withLEDEndIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDEndIndex + 1)); // 28
            } else {
                leds.setControl(
                        Settings.LED.solidColorRequest
                                .withLEDStartIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDEndIndex + 1) // 28 (MAX)
                                .withLEDEndIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDEndIndex + 1)); // 28
            }

        } else if (!isRightLLDead) {
            if (usingFade) {
                leds.setControl(
                        Settings.LED.singleFadeAnimation
                                .withLEDStartIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDStartIndex) // 23
                                .withLEDEndIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDEndIndex + 1)); // 28
            } else {
                leds.setControl(
                        Settings.LED.solidColorRequest
                                .withLEDStartIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDStartIndex) // 23
                                .withLEDEndIndex(Settings.LED.RIGHT_DEAD_STRIP.LEDEndIndex + 1)); // 28
            }
        }

        // LEFT LL
        if (isLeftLLDead) {
            leds.setControl(Settings.LED.LEFT_DEAD_STRIP
                    .withColor(Settings.LED.LLDEAD));

            if (usingFade) {
                leds.setControl(
                        Settings.LED.singleFadeAnimation
                                .withLEDStartIndex(Settings.LED.LEFT_DEAD_STRIP.LEDEndIndex + 1) // 14
                                .withLEDEndIndex(Settings.LED.LEFT_DEAD_STRIP.LEDEndIndex + 2)); // 15
            } else {
                leds.setControl(
                        Settings.LED.solidColorRequest
                                .withLEDStartIndex(Settings.LED.LEFT_DEAD_STRIP.LEDEndIndex + 1) // 14
                                .withLEDEndIndex(Settings.LED.LEFT_DEAD_STRIP.LEDEndIndex + 2)); // 15
            }

        } else if (!isLeftLLDead) {
            if (usingFade) {
                leds.setControl(
                        Settings.LED.singleFadeAnimation
                                .withLEDStartIndex(Settings.LED.LEFT_DEAD_STRIP.LEDStartIndex) // 9
                                .withLEDEndIndex(Settings.LED.LEFT_DEAD_STRIP.LEDEndIndex + 2)); // 15
            } else {
                leds.setControl(
                        Settings.LED.solidColorRequest
                                .withLEDStartIndex(Settings.LED.LEFT_DEAD_STRIP.LEDStartIndex) // 9
                                .withLEDEndIndex(Settings.LED.LEFT_DEAD_STRIP.LEDEndIndex + 2)); // 15
            }
        }

        // BACK LL
        if (isBackLLDead) {
            leds.setControl(Settings.LED.BACK_DEAD_STRIP
                    .withColor(Settings.LED.LLDEAD));

            if (usingFade) {
                leds.setControl(
                        Settings.LED.singleFadeAnimation
                                .withLEDStartIndex(Settings.LED.BACK_DEAD_STRIP.LEDEndIndex + 1) // 21
                                .withLEDEndIndex(Settings.LED.BACK_DEAD_STRIP.LEDEndIndex + 2)); // 22
            } else {
                leds.setControl(
                        Settings.LED.solidColorRequest
                                .withLEDStartIndex(Settings.LED.BACK_DEAD_STRIP.LEDEndIndex + 1) // 21
                                .withLEDEndIndex(Settings.LED.BACK_DEAD_STRIP.LEDEndIndex + 2)); // 22
            }
        } else if (!isBackLLDead) {

            if (usingFade) {
                leds.setControl(
                        Settings.LED.singleFadeAnimation
                                .withLEDStartIndex(Settings.LED.BACK_DEAD_STRIP.LEDStartIndex) // 16
                                .withLEDEndIndex(Settings.LED.BACK_DEAD_STRIP.LEDEndIndex + 2)); // 22
            } else {
                leds.setControl(
                        Settings.LED.solidColorRequest
                                .withLEDStartIndex(Settings.LED.BACK_DEAD_STRIP.LEDStartIndex) // 16
                                .withLEDEndIndex(Settings.LED.BACK_DEAD_STRIP.LEDEndIndex + 2)); // 22
            }
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