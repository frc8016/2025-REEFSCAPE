package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepClimb extends SubsystemBase {
  private final SparkMax climbLeft = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax climbRight = new SparkMax(0, MotorType.kBrushless);
}
