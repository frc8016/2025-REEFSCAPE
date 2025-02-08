package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final SparkMax m_outtakeLeft = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax m_outtakeRight = new SparkMax(0, MotorType.kBrushless);

public void intake (double speed) {
        m_outtakeLeft.set(speed);
        m_outtakeRight.set(speed);
    }

public void score (double speed) {
        m_outtakeLeft.set(speed);
        m_outtakeRight.set(-speed);
    }
}
 
 
