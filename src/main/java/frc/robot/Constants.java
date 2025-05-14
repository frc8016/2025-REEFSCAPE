// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class ElevatorConstants {

        public static final double P_VALUE = 0.35;
        public static final double I_VALUE = 0.0;
        public static final double D_VALUE = 0.3;
        public static final double OUTPUTRANGE_MIN_VALUE = -1.0;
        public static final double OUTPUTRANGE_MAX_VALUE = 1.0;

        public static final double MAX_VEL_RPM = 950;

        public static final double MAX_ACCEL_RPM = MAX_VEL_RPM / 0.7;

        public static final double ALLOWED_SETPOINT_ERROR = 0.1;
        public static final double ELEVATOR_FORWORD_SOFTLIMIT = 50;
        public static final double ELEVATOR_REVERSE_SOFTLIMIT = 0;
        public static final int MAX_CURRENT_LIMIT = 50;
        public static final double ELEVATOR_FEEDFORWORD_CONSTANT = 0.2;
    }

    public static class SetPointConstants {
        public static double LEVEL4 = (16.0);
        public static double LEVEL3 = (9.0);
        public static double LEVEL2 = (4.5);
        public static double TROUGH = (2.0);

        public static double LEVEL1 = (0.0);

    }

    public static class AlgaeIntakeConstants {

        public static final double P_VALUE = 0.4;
        public static final double I_VALUE = 0.0;
        public static final double D_VALUE = 0.2;
        public static final Angle ALLOWED_SETPOINT_ERROR = Degrees.of(1);
        public static final double ALGEAINTAKE_FORWORD_SOFTLIMIT = 1;
        public static final double ALGEAINTAKE_REVERSE_SOFTLIMIT = 0;
        public static final int MAX_CURRENT_LIMIT = 25;
        public static final Double UP_POSITION = -0.02;
        public static final Double DOWN_POSITION = -3.0;
        public static final Double OUTPUTRANGE_MIN_VALUE = -1.0;
        public static final Double OUTPUTRANGE_MAX_VALUE = 1.0;
    }

    public static class DeepClimbConstants {
        public static final double DEGREE_PER_REVOLUTION = 360;
        public static final double VELOCITY_CONVERSHION_FACTOR = DEGREE_PER_REVOLUTION / 60;
        public static final double P_VALUE = 0.01;
        public static final double I_VALUE = 0.00001;
        public static final double D_VALUE = 0.2;
        public static final double P_VALUE_VELOCITY = 0.1;
        public static final double I_VALUE_VELOCITY = 0.0;
        public static final double D_VALUE_VELOCITY = 0.0;
        public static final double OUTPUTRANGE_MIN_VALUE = 0.0;
        public static final double OUTPUTRANGE_MAX_VALUE = 0.0;
        public static final double MAX_VEL = 0.4;
        public static final double MAX_ACCELERATION = MAX_VEL / .5;
        public static final Angle ALLOWED_SETPOINT_ERROR = Degrees.of(1);
        public static final int MAX_CURRENT_LIMIT = 40;
        public static final double FORWORD_SOFTLIMIT = 360;
        public static final double REVERSE_SOFTLIMIT = -360;

    }

    public static class DriveSpeedConstants {
        public static final double SLOW_SPEED_DIVISOR = 3;
    }

    public static class VisionConstants {
        public static final boolean USE_VISION = true; // IMPORTANT we set this to true when useing vision and faluse
                                                       // when we dont (this will effect all vision uses)
        public static final double VISION_MAX_DIST = 3;
        public static final double MAX_TAG_AMBIGUITY = 0.15;

        public static final String LOWER_RIGHT_CAMERA_NAME = "ArducamOV2311Cam1";
        public static final String LOWER_LEFT_CAMERA_NAME = "ArducamOV2311Cam2";

        public static final Transform3d LOWER_RIGHT_CAMERA_POSE = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(12.5), // x: forward positive
                        Units.inchesToMeters(-10.75), // y: left positive
                        Units.inchesToMeters(9)), // z: up positive
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(45)));

        public static final Transform3d LOWER_LEFT_CAMERA_POSE = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(12.625), // x: forward positive
                        Units.inchesToMeters(10.25), // y: left positive
                        Units.inchesToMeters(9)), // z: up positive
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-45)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1,
        // 1, 2);
    }

    public static class PathfindToScoreConstants {
        public static final Map<Pose2d, String> BLUE_SCORING_POSITIONS = new HashMap<>(14) {
            {
                // put(new Pose2d(2.116, 6.692, new Rotation2d()), "CST");
                // put(new Pose2d(2.066, 1.379, new Rotation2d()), "CSB");
                put(new Pose2d(5.414, 5.607, new Rotation2d()), "P1");
                put(new Pose2d(6.339, 4.037, new Rotation2d()), "P2");
                put(new Pose2d(5.484, 2.487, new Rotation2d()), "P3");
                put(new Pose2d(3.582, 3.582, new Rotation2d()), "P4");
                put(new Pose2d(2.646, 4.030, new Rotation2d()), "P5");
                put(new Pose2d(3.609, 5.629, new Rotation2d()), "P6");
            }
        };

        public static final Map<Pose2d, String> RED_SCORING_POSITIONS = new HashMap<>(14) {
            {
                // put(new Pose2d(15.432, 1.360, new Rotation2d()), "CST");
                // put(new Pose2d(15.482, 6.673, new Rotation2d()), "CSB");
                put(new Pose2d(12.134, 2.445, new Rotation2d()), "P1");
                put(new Pose2d(11.209, 4.015, new Rotation2d()), "P2");
                put(new Pose2d(12.064, 5.565, new Rotation2d()), "P3");
                put(new Pose2d(13.966, 4.470, new Rotation2d()), "P4");
                put(new Pose2d(14.902, 4.022, new Rotation2d()), "P5");
                put(new Pose2d(13.939, 2.423, new Rotation2d()), "P6");
            }
        };

        public enum Direction {
            LEFT,
            RIGHT
        }

        public static PathConstraints constraints = new PathConstraints(
                5.210, 7.1,
                Units.degreesToRadians(540), Units.degreesToRadians(1851));
    }
}
