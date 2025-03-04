package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
        // #Note that they changed the spark max name from "CANSparkMax" to "SparkMax"
        // Declare devices
        //
        private final SparkMax m_elevatorLeft = new SparkMax(13, MotorType.kBrushless);
        private final SparkMax m_elevatorRight = new SparkMax(15, MotorType.kBrushless);
        private final SparkMaxConfig m_sharedconfig = new SparkMaxConfig();
        private final SparkMaxConfig m_leftconfig = new SparkMaxConfig();
        private final SparkMaxConfig m_rightconfig = new SparkMaxConfig();
        private final SparkClosedLoopController m_rightClosedLoopController = m_elevatorRight.getClosedLoopController();
        private double position;
        // Creates the feedforward control for the elevator

        public Elevator() {
                m_sharedconfig.encoder
                                .positionConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION)
                                .velocityConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION / 60);

                m_sharedconfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(ElevatorConstants.P_VALUE, ClosedLoopSlot.kSlot0)
                                .i(ElevatorConstants.I_VALUE, ClosedLoopSlot.kSlot0)
                                .d(ElevatorConstants.D_VALUE, ClosedLoopSlot.kSlot0)
                                .outputRange(ElevatorConstants.OUTPUTRANGE_MIN_VALUE,
                                                ElevatorConstants.OUTPUTRANGE_MAX_VALUE)
                                .p(ElevatorConstants.P_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                                .i(ElevatorConstants.I_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                                .d(ElevatorConstants.D_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                                // https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
                                .velocityFF(ElevatorConstants.FEEDFORWARD_VALUE, ClosedLoopSlot.kSlot1)
                                .outputRange(ElevatorConstants.OUTPUTRANGE_MIN_VALUE,
                                                ElevatorConstants.OUTPUTRANGE_MAX_VALUE,
                                                ClosedLoopSlot.kSlot1);

                m_sharedconfig.closedLoop.maxMotion
                                .maxVelocity(ElevatorConstants.MAX_VEL.in(MetersPerSecond))
                                .maxAcceleration(ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond))
                                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                                .allowedClosedLoopError(ElevatorConstants.ALLOWED_SETPOINT_ERROR.in(Meters));

                m_rightconfig
                                .apply(m_sharedconfig)
                                .inverted(true)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(ElevatorConstants.MAX_CURRENT_LIMIT);

                m_leftconfig
                                .apply(m_sharedconfig)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(ElevatorConstants.MAX_CURRENT_LIMIT)
                                .follow(m_elevatorRight, true);

                m_leftconfig.softLimit
                                .forwardSoftLimit(ElevatorConstants.ELEVATOR_FORWORD_SOFTLIMIT)
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(ElevatorConstants.ELEVATOR_REVERSE_SOFTLIMIT)
                                .reverseSoftLimitEnabled(true);

                m_rightconfig.softLimit
                                .forwardSoftLimit(ElevatorConstants.ELEVATOR_FORWORD_SOFTLIMIT)
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(ElevatorConstants.ELEVATOR_REVERSE_SOFTLIMIT)
                                .reverseSoftLimitEnabled(true);

                m_elevatorLeft.configure(m_leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                m_elevatorRight.configure(m_rightconfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }

        public void setSpeed(double speed) {
                m_rightClosedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);

        }

        public void setPosition(double position) {
                double ff = 0;
                if (position > .5){
                        ff = -0.3;
                }
                m_rightClosedLoopController.setReference(position, ControlType.kPosition,
                                ClosedLoopSlot.kSlot0, ff);

                this.position = position;
                System.out.println("set position" + position);
        }

       

        public Command goToSetPointCommand1(double position) {
                return this.runOnce(() -> this.setPosition(ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0));
        }

        public Command goToSetPointCommand(double position) {
                return new InstantCommand(() -> this.setPosition(ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0));

        }

        public Command setSpeedCommand(double speed) {
                return this.run(() -> this.setSpeed(speed));
        }
       
       

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Encoder position",  m_elevatorRight.getEncoder().getPosition());
                SmartDashboard.putNumber("Encoder velocity", m_elevatorRight.getEncoder().getVelocity());
                SmartDashboard.putNumber("Duty Cycle", m_elevatorRight.getAppliedOutput());
                SmartDashboard.putNumber("Bus voltage", m_elevatorRight.getBusVoltage());
                SmartDashboard.putNumber("Output current", m_elevatorRight.getOutputCurrent());

        }

}
