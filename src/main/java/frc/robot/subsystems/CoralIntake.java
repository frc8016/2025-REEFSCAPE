package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final SparkMax m_outtakeLeft = new SparkMax(14, MotorType.kBrushless);
    private final SparkMax m_outtakeRight = new SparkMax(16, MotorType.kBrushless);
    private final DigitalInput m_beamBreak = new DigitalInput(0);

    public void runRollers(double speed) {
        m_outtakeLeft.set(speed);
        m_outtakeRight.set(-speed);
    }

    public boolean isBeamBroken() {
        return m_beamBreak.get();
    }

    public BooleanSupplier notBroken(){
        return (() -> m_beamBreak.get() == true);

    }

    public BooleanSupplier isBroken(){
        return (() -> m_beamBreak.get() == false);
    }

    public boolean getAsBoolean(){
        return isBroken().getAsBoolean();
    }

    @Override
    public void periodic(){
        isBroken();
        SmartDashboard.putBoolean("Beam Break ", isBeamBroken());
        SmartDashboard.putBoolean("bb", getAsBoolean());
    }
    
}
