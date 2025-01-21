package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorLeft = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax elevatorRight = new SparkMax(0, MotorType.kBrushless);
}
