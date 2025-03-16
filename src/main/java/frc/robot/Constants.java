// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

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
        // private static final Distance ELEVATOR_SPROCKET_DIA = Centimeters.of(4.445);
        // private static final double GEARBOX_RATIO = 3.0;
        // Multipier to convert rotatons of motor to meters of travel for elevator
        // public static final double POS_CONVERSION_FACTOR = Math.PI *
        // ELEVATOR_SPROCKET_DIA.in(Meters) / GEARBOX_RATIO;
        // Multiplier to convert RPM of motor to m/s of elevator;
        // public static final double VELOCITY_CONVERSION_FACTOR = POS_CONVERSION_FACTOR
        // / 60;
        public static final double P_VALUE = 0.2;
        public static final double I_VALUE = 0.0;
        public static final double D_VALUE = 0.0;
        public static final double OUTPUTRANGE_MIN_VALUE = -1.0;
        public static final double OUTPUTRANGE_MAX_VALUE = 1.0;

        public static final double MAX_VEL_RPM = 800;

        public static final double MAX_ACCEL_RPM = MAX_VEL_RPM / 0.7;

        public static final double ALLOWED_SETPOINT_ERROR = 0.3;
        public static final double ELEVATOR_FORWORD_SOFTLIMIT = 50;
        public static final double ELEVATOR_REVERSE_SOFTLIMIT = 0;
        public static final int MAX_CURRENT_LIMIT = 50;
        public static final double ELEVATOR_FEEDFORWORD_CONSTANT = 0.2;
    }

    public static class SetPointConstants {
        public static double LEVEL4 = (16.0);
        public static double LEVEL3 = (9.0);
        public static double LEVEL2 = (4.5);
        public static double LEVEL1 = (0.0);

    }

    public static class AlgaeIntakeConstants {

        public static final double P_VALUE = 0.05;
        public static final double I_VALUE = 0.0;
        public static final double D_VALUE = 0.15;
        public static final Angle ALLOWED_SETPOINT_ERROR = Degrees.of(1);
        public static final double ALGEAINTAKE_FORWORD_SOFTLIMIT = 1;
        public static final double ALGEAINTAKE_REVERSE_SOFTLIMIT = 0;
        public static final int MAX_CURRENT_LIMIT = 30;
        public static final Double UP_POSITION = 5.0;
        public static final Double DOWN_POSITION = 0.0;
        public static final Double OUTPUTRANGE_MIN_VALUE = -1.0;
        public static final Double OUTPUTRANGE_MAX_VALUE = 1.0;
    }

    public static class DeepClimbConstants {
        public static final double DEGREE_PER_REVOLUTION = 360;
        public static final double VELOCITY_CONVERSHION_FACTOR = DEGREE_PER_REVOLUTION / 60;
        public static final double P_VALUE = .3;
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

    public static class VisionConstants {
        public static final boolean USE_VISION = true;
        public static final String CAMERA_NAME = "YOUR CAMERA NAME";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        // public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4,
        // 4, 8);
        // public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5,
        // 0.5, 1);
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1, 1, 2);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.15, 0.15, 0.5);
    }
}
