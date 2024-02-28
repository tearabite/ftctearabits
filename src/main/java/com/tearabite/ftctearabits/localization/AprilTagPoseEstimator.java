package com.tearabite.ftctearabits.localization;

import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.Pose2d;

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

@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AprilTagPoseEstimator {

    /*
    * This class is used to estimate the pose of the robot using the AprilTagDetection object.
    * The math done here uses only trigonometric functions so that it can be better understood
    * by students without knowledge of linear algebra or matrix operations.
    * The math done hre can be visualized using the following Desmos graph:
    * https://www.desmos.com/calculator/n2iyatwssg
     */

    @Getter @Setter @Builder.Default  private Pose2d robotOffset = new Pose2d(0,0,0);

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
                fieldOrientation.w);
    }

    /**
     * Estimates the pose of the robot using the AprilTagDetection object.
     * @param yaw The rotation about Z of the AprilTag within the camera's frame.
     * @param bearing The angle between line projected to center of the april tag and the camera's center projection line.
     * @param range The distance from the center of the camera lens to the center of the AprilTag.
     * @param Tx The x position (in FTC field coordinates) of the AprilTag
     * @param Ty The y position (in FTC field coordinates) of the AprilTag
     * @param Ta The a component of the quaternion describing the orientation of the AprilTag
     * @param Tb The b component of the quaternion describing the orientation of the AprilTag
     * @param Tc The c component of the quaternion describing the orientation of the AprilTag
     * @param Td The d component of the quaternion describing the orientation of the AprilTag
     * @return The estimated pose of the robot. Heading is reported in radians.
     */
    private Pose2d estimatePose(double yaw, double bearing, double range, double Tx, double Ty, double Ta, double Tb, double Tc, double Td) {
        double aprilTagHeading = getTYaw(Ta, Tb, Tc, Td);
        double cameraRotation = ((PI / 2) - aprilTagHeading + yaw);
        double cx = Tx + range * cos(bearing) * cos(cameraRotation) + range * sin(bearing) * sin(cameraRotation);
        double cy = Ty + range * cos(bearing) * sin(cameraRotation) - range * sin(bearing) * cos(cameraRotation);

        double offsetX = this.robotOffset.position.x;
        double offsetY = this.robotOffset.position.y;
        double cameraRotationOnRobot = this.robotOffset.heading.toDouble();
        double robotRotation = (PI / 2) + aprilTagHeading + yaw - cameraRotationOnRobot;
        double rx = cx + offsetX * cos(robotRotation) - offsetY * sin(robotRotation);
        double ry = cy + offsetX * sin(robotRotation) + offsetY * cos(robotRotation);
        double rh = (yaw - cameraRotationOnRobot + 2 * PI) % (2 * PI);

        return new Pose2d(rx, ry, rh);
    }

    private static double getTPitch(double a, double b, double c, double d) {
        return asin(2 * ((a * d) - (b * c)));
    }

    private static double getTRoll(double a, double b, double c, double d) {
        return atan2(2 * ((a * c) + (b * d)), 1 - 2 * (pow(c, 2) + pow(d, 2)));
    }

    private static double getTYaw(double a, double b, double c, double d) {
        return atan2(2 * ((a * b) + (c * d)), 1 - 2 * ((pow(b, 2) + pow(c, 2))));
    }
}
