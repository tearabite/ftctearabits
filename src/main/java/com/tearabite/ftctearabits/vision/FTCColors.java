package com.tearabite.ftctearabits.vision;

import org.opencv.core.Scalar;
public class FTCColors {
    public static Scalar FTC_RED_LOWER = new Scalar(165, 80, 80);
    public static Scalar FTC_RED_UPPER = new Scalar(15, 255, 255);
    public static ScalarRange FTC_RED_RANGE_1 = new ScalarRange(new Scalar(180, FTC_RED_UPPER.val[1], FTC_RED_UPPER.val[2]), FTC_RED_LOWER);
    public static ScalarRange FTC_RED_RANGE_2 = new ScalarRange(FTC_RED_UPPER, new Scalar(0, FTC_RED_LOWER.val[1], FTC_RED_LOWER.val[2]));
    public static Scalar FTC_BLUE_LOWER = new Scalar(75, 40, 80);
    public static Scalar FTC_BLUE_UPPER = new Scalar(120, 255, 255);
    public static ScalarRange FTC_BLUE_RANGE = new ScalarRange(FTC_BLUE_UPPER, FTC_BLUE_LOWER);
    public static Scalar FTC_WHITE_LOWER = new Scalar(0, 0, 40);
    public static Scalar FTC_WHITE_UPPER = new Scalar(180, 30, 255);
}