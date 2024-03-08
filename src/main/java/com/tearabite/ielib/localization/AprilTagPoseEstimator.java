package com.tearabite.ielib.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.security.InvalidParameterException;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AprilTagPoseEstimator extends AprilTagPoseEstimatorCore {

    /*
    * This class is used to estimate the pose of the robot using the AprilTagDetection object.
    * The math done here uses only trigonometric functions so that it can be better understood
    * by students without knowledge of linear algebra or matrix operations.
    * The math done hre can be visualized using the following Desmos graph:
    * https://www.desmos.com/calculator/n2iyatwssg
     */

    @Setter private Pose2d robotOffset;

    /**
     * Estimates the pose of the robot using the AprilTagDetection object.
     * @param detection The AprilTagDetection object
     * @return The estimated pose of the robot
     */
    public Pose2d estimatePose(AprilTagDetection detection) {
        if (detection == null || detection.metadata == null || detection.metadata.fieldPosition == null || detection.ftcPose == null) {
            throw new InvalidParameterException();
        }

        AprilTagPoseFtc ftcPose = detection.ftcPose;
        VectorF fieldPosition = detection.metadata.fieldPosition;
        Quaternion fieldOrientation = detection.metadata.fieldOrientation;
        return estimatePose(
                Math.toRadians(ftcPose.yaw),
                Math.toRadians(ftcPose.bearing),
                ftcPose.range,
                fieldPosition.get(0),
                fieldPosition.get(1),
                fieldOrientation.x,
                fieldOrientation.y,
                fieldOrientation.z,
                fieldOrientation.w,
                this.robotOffset);
    }
}
