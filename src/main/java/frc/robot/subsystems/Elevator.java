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
   private final SparkMax motor_elevatorLeft = new SparkMax(0, MotorType.kBrushless); //NEEDS ID
   private final SparkMax motor_elevatorRight = new SparkMax(0, MotorType.kBrushless); //NEEDS ID
   private final Encoder relativeEncoder = new Encoder(null, null); //NEEDS ID
   private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0); //NEEDS ID
   SparkMaxConfig config = new SparkMaxConfig();
   // Creates the feedforward control for the elevator
   // gains
   float ks = 0;//static
   float kg = 0;//gravity
   float kv = 0;//velocity
   float ka = 0;//acceleration
   private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ks, kg, kv, ka);

   public Elevator() {
     float kp = 1;//proportional gain,
     float kd = 0.5;//derivitive gain
     float ki = 0.5;//integral gain, 
     float max_rate = 5;//max rate in m/s
     float max_accel = 10;//max accel in m/s^2
     new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(max_rate, max_accel));

     relativeEncoder.reset();
     float dPulse = 0.01;//distance per pulse in m
     relativeEncoder.setDistancePerPulse(dPulse);
     // configure motors :)
     // configureMotors();
     // m_elevatorLeft.configure(config,
     // ResetMode.kResetSafeParameters
     // PersistMode.kPersistParameters);
   }
}
