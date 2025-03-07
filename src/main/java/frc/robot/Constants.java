// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
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
        public static final double METERS_PER_REVOLUTION = 0.1397 / 3.0;
        public static final double P_VALUE = 1.475;
        public static final double I_VALUE = 0.0;
        public static final double D_VALUE = 0.15;
        public static final double P_VALUE_VELOCITY = 0.5;
        public static final double I_VALUE_VELOCITY = 0.0;
        public static final double D_VALUE_VELOCITY = 0.0;
        public static final double FEEDFORWARD_VALUE = 1.0 / 473.0;
        public static final double OUTPUTRANGE_MIN_VALUE = -1.0;
        public static final double OUTPUTRANGE_MAX_VALUE = 1.0;
        public static final LinearVelocity MAX_VEL = MetersPerSecond.of(0.4);
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(.4);
        public static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(1);
        public static final double ELEVATOR_FORWORD_SOFTLIMIT = 50;
        public static final double ELEVATOR_REVERSE_SOFTLIMIT = 0;
        public static final int MAX_CURRENT_LIMIT = 50;

    }

    public static class SetPointConstants {

        public static Distance LEVEL4 = Centimeters.of(84.0);
        public static Distance LEVEL3 = Centimeters.of(42.0);
        public static Distance LEVEL2 = Centimeters.of(21.0);
        public static Distance LEVEL1 = Centimeters.of(0.0);
    }

    public static class AlgaeIntakeConstants {
        public static final double DEGREE_PER_REVOLUTION = 360;
        public static final double VELOCITY_CONVERSHION_FACTOR = DEGREE_PER_REVOLUTION / 60;
        public static final double P_VALUE = 1.475;
        public static final double I_VALUE = 0.0;
        public static final double D_VALUE = 0.15;
        public static final double P_VALUE_VELOCITY = 0.5;
        public static final double I_VALUE_VELOCITY = 0.0;
        public static final double D_VALUE_VELOCITY = 0.0;
        public static final double OUTPUTRANGE_MIN_VALUE = 0.0;
        public static final double OUTPUTRANGE_MAX_VALUE = 0.0;
        public static final double MAX_VEL = 0.4; // dont use unless sure
        public static final double MAX_ACCELERATION = MAX_VEL / .5;// dont use unless sure
        public static final Angle ALLOWED_SETPOINT_ERROR = Degrees.of(1);
        public static final double ALGEAINTAKE_FORWORD_SOFTLIMIT = 1;
        public static final double ALGEAINTAKE_REVERSE_SOFTLIMIT = 0;

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

    }
}