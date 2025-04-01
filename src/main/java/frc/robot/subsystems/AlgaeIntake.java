package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.security.AlgorithmConstraints;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.ElevatorConstants;

//test
public class AlgaeIntake extends SubsystemBase {
    // v- needed
    private final SparkMax m_algaeMotor = new SparkMax(19, MotorType.kBrushless); // double check id
    private final SparkMaxConfig m_algaeMotorconfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_algaeClosedLoopController = m_algaeMotor.getClosedLoopController();
    // private final DigitalInput m_beamBreak = new DigitalInput(0);

    public AlgaeIntake() {
        m_algaeMotorconfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(AlgaeIntakeConstants.P_VALUE)
                .i(AlgaeIntakeConstants.I_VALUE)
                .d(AlgaeIntakeConstants.D_VALUE)
                .outputRange(AlgaeIntakeConstants.OUTPUTRANGE_MIN_VALUE, AlgaeIntakeConstants.OUTPUTRANGE_MAX_VALUE);
        m_algaeMotorconfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(AlgaeIntakeConstants.MAX_CURRENT_LIMIT);

        m_algaeMotor.configure(m_algaeMotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // sets some softlimits
        // m_algaeMotorconfig.softLimit
        // .forwardSoftLimit(AlgaeIntakeConstants.ALGEAINTAKE_FORWORD_SOFTLIMIT)
        // .forwardSoftLimitEnabled(true)
        // .reverseSoftLimit(AlgaeIntakeConstants.ALGEAINTAKE_REVERSE_SOFTLIMIT)
        // .reverseSoftLimitEnabled(true);

    }

    // lets driverstation set a position
    public void setPosition(double position) {
        System.out.println("POSITION" + position);
        m_algaeClosedLoopController.setReference(position, ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
    }

    // makes a command we can use
    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> this.setPosition(position));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algea Position ", m_algaeMotor.getEncoder().getPosition());
        //SmartDashboard.putNumber("Algae Velocity", m_algaeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("AlgeaTemp ", m_algaeMotor.getMotorTemperature());

       
    }

}
