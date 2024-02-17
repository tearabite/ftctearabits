package com.tearabite.ftctearabits.localization;

import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.tan;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.security.InvalidParameterException;

public class AprilTagPoseEstimator {

    private final Pose2d robotOffset;

    public AprilTagPoseEstimator() {
        this(new Pose2d(new Vector2d(0, 0), 0));
    }

    public AprilTagPoseEstimator(Pose2d robotOffset) {
        this.robotOffset = robotOffset;
    }


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

    private Pose2d estimatePose(double yaw, double bearing, double range, double Tx, double Ty, double Ta, double Tb, double Tc, double Td) {
        double aprilTagHeading = getTYaw(Ta, Tb, Tc, Td);
        double cameraRotation = ((PI / 2) - aprilTagHeading + yaw);
        double cx = Tx + range * cos(bearing) * cos(cameraRotation) + range * sin(bearing) * sin(cameraRotation);
        double cy = Ty + range * cos(bearing) * sin(cameraRotation) - range * sin(bearing) * cos(cameraRotation);

        double aprilTagX = this.robotOffset.position.x;
        double aprilTagY = this.robotOffset.position.y;
        double cameraRotationOnRobot = this.robotOffset.heading.toDouble();
        double robotRotation = (PI / 2) + aprilTagHeading + yaw - cameraRotationOnRobot;
        double rx = cx + aprilTagX * cos(robotRotation) - aprilTagY * sin(robotRotation);
        double ry = cy + aprilTagX * sin(robotRotation) + aprilTagY * cos(robotRotation);
        double rh = tan(yaw - cameraRotationOnRobot);

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
