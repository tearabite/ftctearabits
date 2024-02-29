package com.tearabite.ftctearabits.vision;

import org.opencv.core.Scalar;

import lombok.AllArgsConstructor;
import lombok.Data;

/**
 * A class for specifying an upper and lower bound for a color range.
 */
@Data
@AllArgsConstructor
public class ScalarRange {
    private Scalar upper;
    private Scalar lower;
}