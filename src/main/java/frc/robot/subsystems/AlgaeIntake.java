package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.security.AlgorithmConstraints;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax m_algaeMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkMaxConfig m_algaeMotorconfig = new SparkMaxConfig();

    public AlgaeIntake() {
        // m_algaeMotorconfig.encoder
        m_algaeMotorconfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                .p(AlgaeIntakeConstants.P_VALUE, ClosedLoopSlot.kSlot0)
                .i(AlgaeIntakeConstants.I_VALUE, ClosedLoopSlot.kSlot0)
                .d(AlgaeIntakeConstants.D_VALUE, ClosedLoopSlot.kSlot0)
    }
}
