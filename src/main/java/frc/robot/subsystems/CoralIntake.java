package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  private final SparkMax coralLeft = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax coralRight = new SparkMax(0, MotorType.kBrushless);
}
