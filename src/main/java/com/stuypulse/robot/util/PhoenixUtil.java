package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.BaseStatusSignal;

public class PhoenixUtil {
    public static ArrayList<BaseStatusSignal> allSignals = new ArrayList<BaseStatusSignal>();

    public PhoenixUtil() {}

    public static void registerSignals(BaseStatusSignal... signals) {
        allSignals.addAll(Arrays.asList(signals));
    }


    public static void refreshAll() {
        BaseStatusSignal.refreshAll(allSignals);
    }
}
