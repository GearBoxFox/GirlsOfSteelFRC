package com.gos.crescendo2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotExtrinsics {
    // Robot Size
    public static final double ROBOT_WIDTH = Units.inchesToMeters(25);
    public static final double ROBOT_LENGTH = Units.inchesToMeters(25);

    // Camera Extrinsics
    public static final Transform3d ROBOT_TO_CAM_OBJECT_DETECTION = new Transform3d(
        new Translation3d(
            0,
            0,
            .20), // 20cm off the ground
        new Rotation3d(
            Math.toRadians(0),
            Math.toRadians(11),
            Math.toRadians(3)));

    public static final Transform3d ROBOT_TO_CAMERA_APRIL_TAGS_R = new Transform3d(
        new Translation3d(
            RobotExtrinsics.ROBOT_WIDTH / 2 - Units.inchesToMeters(12.25), // 11.75 inches from back
            -(RobotExtrinsics.ROBOT_LENGTH / 2 - Units.inchesToMeters(1.75)), // 1.75 inches from side
            .235),
        new Rotation3d(
            Math.toRadians(180),
            Math.toRadians(-25),
            Math.toRadians(-108))
    );
    public static final Transform3d ROBOT_TO_CAMERA_APRIL_TAGS_L = new Transform3d(
        new Translation3d(
            RobotExtrinsics.ROBOT_WIDTH / 2 - Units.inchesToMeters(11.75), // 11.75 inches from front
            RobotExtrinsics.ROBOT_LENGTH / 2 - Units.inchesToMeters(1.5), // 2 inches from side
            .235),
        new Rotation3d(
            Math.toRadians(180),
            Math.toRadians(-19.2),
            Math.toRadians(100))
    );
    public static final Transform3d ROBOT_TO_CAMERA_APRIL_TAGS_CB = new Transform3d(
        new Translation3d(
            -(RobotExtrinsics.ROBOT_WIDTH / 2 - Units.inchesToMeters(2.5)), // 2.5 inches from back
            0, // Directly Center
            .235),
        new Rotation3d(
            Math.toRadians(180),
            Math.toRadians(-40),
            Math.toRadians(180))
    );


    // Important Robot Poses
    public static final Pose2d STARTING_POSE_AMP_SUBWOOFER = new Pose2d(0.6933452953924437, 6.686887667641241, Rotation2d.fromDegrees(60));
    public static final Pose2d STARTING_POSE_MIDDLE_SUBWOOFER = new Pose2d(1.34, 5.55, Rotation2d.fromDegrees(0));
    public static final Pose2d STARTING_POSE_SOURCE_SUBWOOFER = new Pose2d(0.6933452953924437, 4.403274541213847, Rotation2d.fromDegrees(-60));
    public static final Pose2d SCORE_IN_AMP_POSITION = new Pose2d(1.84, 7.8, Rotation2d.fromDegrees(90));

    // Important Aiming Poses
    public static final Pose2d FULL_FIELD_FEEDING_AIMING_POINT = new Pose2d(FieldConstants.AMP_CENTER, Rotation2d.fromDegrees(0));

}
