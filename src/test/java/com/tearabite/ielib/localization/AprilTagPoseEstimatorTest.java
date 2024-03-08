package com.tearabite.ielib.localization;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.fail;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.security.InvalidParameterException;
import java.util.stream.Stream;

class AprilTagPoseEstimatorTest {

    private static final AprilTagMetadata metadata = new AprilTagMetadata(
            2,
            "testTag",
            0,
            new VectorF(60.25f, 35.41f, 4f), DistanceUnit.INCH,
            new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0));

    @Test
    public void estimatePose_null_throws() {
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator();
        assertThrows(InvalidParameterException.class, () -> estimator.estimatePose(null));
    }

    @ParameterizedTest
    @MethodSource("provideEstimatePosTestValues")
    public void estimatePos_notNull_returnsPose(AprilTagMetadata metadata, AprilTagPoseFtc poseFtc, Pose2d robotOffsets, Pose2d expectedPose) {
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(robotOffsets);
        AprilTagDetection detection = new AprilTagDetection(
                1,
                0,
                0,
                null,
                null,
                metadata,
                poseFtc,
                null,
                0);

        Pose2d estimatedPose = estimator.estimatePose(detection);

        assertIsClose(estimatedPose, expectedPose);
    }

    private static Stream<Arguments> provideEstimatePosTestValues() {
        return Stream.of(
                Arguments.of(
                        metadata,
                        new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 24, 0, 0),
                        new Pose2d(-7.77, 0.505, 0),
                        new Pose2d(28.5, 35.9, 0)),
                Arguments.of(
                        metadata,
                        new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 24, -45, 0),
                        new Pose2d(-7.77, 0.505, 0),
                        new Pose2d(35.5, 52.9, 0)),
                Arguments.of(
                        metadata,
                        new AprilTagPoseFtc(0, 0, 0, -45, 0, 0, 24, -45, 0),
                        new Pose2d(-7.77, 0.505, 0),
                        new Pose2d(55.1, 65.3, Math.PI / 4)),

                Arguments.of(
                        metadata,
                        new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 24, 0, 0),
                        new Pose2d(8.9, -1.5, Math.PI),
                        new Pose2d(27.4, 36.9, Math.PI)),

                Arguments.of(
                        metadata,
                        new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 24, 0, 0),
                        new Pose2d(8.9, -1.5, Math.PI * 3 / 4),
                        new Pose2d(28.9, 30.2, Math.PI * 3 / 4))
        );
    }

    private boolean isClose(double a, double b) {
        return Math.abs(a - b) < 0.1;
    }

    private void assertIsClose(Pose2d a, Pose2d b) {
        boolean isClose = isClose(a.getX(), b.getX())
                && isClose(a.getY(), b.getY())
                && isClose(a.getHeading(), b.getHeading());

        if (!isClose) {
            fail(String.format("Expected (%.1f, %.1f, %.1f) to be close to (%.1f, %.1f, %.1f)",
                    a.getX(), a.getY(), a.getHeading(),
                    b.getX(), b.getY(), b.getHeading()));
        }
    }
}