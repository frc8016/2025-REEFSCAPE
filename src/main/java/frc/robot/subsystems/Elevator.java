package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    // #Note that they changed the spark max name from "CANSparkMax" to "SparkMax"
    // Declare devices
    private final SparkMax m_elevatorLeft = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax m_elevatorRight = new SparkMax(0, MotorType.kBrushless);
    private final Encoder m_relativeEncoder = new Encoder(null, null);
    private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(0);
    private final SparkMaxConfig m_sharedconfig = new SparkMaxConfig();
    private final SparkMaxConfig m_leftconfig = new SparkMaxConfig();
    private final SparkMaxConfig m_rightconfig = new SparkMaxConfig();

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
                .outputRange(ElevatorConstants.OUTPUTRANGE_MIN_VALUE, ElevatorConstants.OUTPUTRANGE_MAX_VALUE)
                .p(ElevatorConstants.P_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                .i(ElevatorConstants.I_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                .d(ElevatorConstants.D_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                // https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
                .velocityFF(ElevatorConstants.FEEDFORWARD_VALUE, ClosedLoopSlot.kSlot1)
                .outputRange(ElevatorConstants.OUTPUTRANGE_MIN_VALUE, ElevatorConstants.OUTPUTRANGE_MAX_VALUE,
                        ClosedLoopSlot.kSlot1);

        m_sharedconfig.closedLoop.maxMotion
                .maxVelocity(ElevatorConstants.MAX_VEL.in(MetersPerSecond))
                .maxAcceleration(ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond))
                .allowedClosedLoopError(ElevatorConstants.ALLOWED_SETPOINT_ERROR.in(Meters));

        m_rightconfig
                .apply(m_sharedconfig)
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);

        m_leftconfig
                .apply(m_sharedconfig)
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .follow(m_elevatorRight);

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
        m_elevatorRight.configure(m_rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
