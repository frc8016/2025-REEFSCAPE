package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  // #Note that they changed the spark max name from "CANSparkMax" to "SparkMax"
  // Declare devices
  private final SparkMax m_elevatorLeft = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax m_elevatorRight = new SparkMax(0, MotorType.kBrushless);
  private final Encoder m_relativeEncoder = new Encoder(null, null);
  private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(0);
  SparkMaxConfig config = new SparkMaxConfig();
  // Creates the feedforward control for the elevator
  private final ElevatorFeedforward m_elevatorFeedForward = new ElevatorFeedforward(0.0, 0.0, 0.0);

  public Elevator() {
    new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    m_relativeEncoder.reset();
    m_relativeEncoder.setDistancePerPulse(0);
    // configure motors :)
    // configureMotors();
    // m_elevatorLeft.configure(config,
    // ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

  }
}
