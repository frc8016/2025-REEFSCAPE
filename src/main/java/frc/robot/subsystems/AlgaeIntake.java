package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.security.AlgorithmConstraints;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlgaeIntake extends SubsystemBase {
        // v- needed
        private final SparkMax m_algaeMotor = new SparkMax(18, MotorType.kBrushless); // double check id 
        private final SparkMaxConfig m_algaeMotorconfig = new SparkMaxConfig();
        private final SparkClosedLoopController m_algaeClosedLoopController = m_algaeMotor.getClosedLoopController();
        private final DigitalInput m_beamBreak = new DigitalInput(0);

        public AlgaeIntake() {
                // converts Rotations values to degree values
                m_algaeMotorconfig.encoder
                                .positionConversionFactor(AlgaeIntakeConstants.DEGREE_PER_REVOLUTION)
                                .velocityConversionFactor(AlgaeIntakeConstants.VELOCITY_CONVERSHION_FACTOR);
                // configs the pid loops
                m_algaeMotorconfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(AlgaeIntakeConstants.P_VALUE, ClosedLoopSlot.kSlot0)
                                .i(AlgaeIntakeConstants.I_VALUE, ClosedLoopSlot.kSlot0)
                                .d(AlgaeIntakeConstants.D_VALUE, ClosedLoopSlot.kSlot0)
                                .outputRange(AlgaeIntakeConstants.OUTPUTRANGE_MIN_VALUE,
                                                AlgaeIntakeConstants.OUTPUTRANGE_MAX_VALUE);
                // configs maxMotion
                m_algaeMotorconfig.closedLoop.maxMotion
                                .maxVelocity(AlgaeIntakeConstants.MAX_VEL)
                                .maxAcceleration(AlgaeIntakeConstants.MAX_ACCELERATION)
                                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                                .allowedClosedLoopError(AlgaeIntakeConstants.ALLOWED_SETPOINT_ERROR.in(Degrees));
                // sets some softlimits
                m_algaeMotorconfig.softLimit
                                .forwardSoftLimit(AlgaeIntakeConstants.ALGEAINTAKE_FORWORD_SOFTLIMIT)
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(AlgaeIntakeConstants.ALGEAINTAKE_REVERSE_SOFTLIMIT)
                                .reverseSoftLimitEnabled(true);

        }

        // lets driverstation set a position
        public void setPosition(double position) {
                m_algaeClosedLoopController.setReference(position, ControlType.kMAXMotionPositionControl,
                                ClosedLoopSlot.kSlot0);
        }

        // makes a command we can use
        public Command goToSetPointCommand(double position) {
                return this.runOnce(() -> this.setPosition(position));

        }
@Override
public void periodic(){
        SmartDashboard.putNumber("Algea Position", m_algaeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Algae Velocity", m_algaeMotor.getEncoder().getVelocity());
}

}
