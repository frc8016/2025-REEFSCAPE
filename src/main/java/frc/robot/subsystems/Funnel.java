package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(20, MotorType.kBrushless);
    private final DigitalInput m_limitSwitch = new DigitalInput(2);


    public void run(double speed){
        m_motor.set(speed);
    }

    public boolean getLimitSwitch(){
        return m_limitSwitch.get();
    }

     public BooleanSupplier triggered(){
        return (() -> m_limitSwitch.get() == false);

    }


    @Override
        public void periodic() {
               SmartDashboard.putBoolean("Funnel", !m_limitSwitch.get());

              
        }
}
