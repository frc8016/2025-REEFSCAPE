package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeepClimbConstants;

public class DeepClimb extends SubsystemBase {
    private final SparkMax m_climbLeft = new SparkMax(18, MotorType.kBrushless);
    private final SparkMax m_climbRight = new SparkMax(17, MotorType.kBrushless);
    private final SparkMax m_climb21 = new SparkMax(21, MotorType.kBrushless);
    private final SparkMaxConfig m_21config = new SparkMaxConfig();
    private final SparkMaxConfig m_sharedclimbconfig = new SparkMaxConfig();
    private final SparkMaxConfig m_climbLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_climbRightConfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_leftClosedLoopController = m_climbLeft.getClosedLoopController();
    public DeepClimb() {

        m_sharedclimbconfig.encoder
                .positionConversionFactor(DeepClimbConstants.DEGREE_PER_REVOLUTION)
                .velocityConversionFactor(DeepClimbConstants.VELOCITY_CONVERSHION_FACTOR);

        m_sharedclimbconfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(DeepClimbConstants.P_VALUE)
                .i(DeepClimbConstants.I_VALUE)
                .d(DeepClimbConstants.D_VALUE)
                .outputRange(DeepClimbConstants.OUTPUTRANGE_MIN_VALUE,
                        DeepClimbConstants.OUTPUTRANGE_MAX_VALUE);

        m_sharedclimbconfig.closedLoop.maxMotion
                .maxVelocity(DeepClimbConstants.MAX_VEL)
                .maxAcceleration(DeepClimbConstants.MAX_ACCELERATION)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .allowedClosedLoopError(DeepClimbConstants.ALLOWED_SETPOINT_ERROR.in(Degrees));

        m_climbLeftConfig
                .apply(m_sharedclimbconfig)
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DeepClimbConstants.MAX_CURRENT_LIMIT);

        m_climbRightConfig
                .apply(m_sharedclimbconfig)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DeepClimbConstants.MAX_CURRENT_LIMIT);
        // .follow(m_climbLeft, false);

        m_21config
                .idleMode(IdleMode.kBrake);
                m_21config.smartCurrentLimit(DeepClimbConstants.MAX_CURRENT_LIMIT);

        // m_climbLeftConfig.softLimit
        // .forwardSoftLimit(DeepClimbConstants.FORWORD_SOFTLIMIT)
        // .forwardSoftLimitEnabled(true)
        // .reverseSoftLimit(DeepClimbConstants.REVERSE_SOFTLIMIT)
        // .reverseSoftLimitEnabled(true);

        // m_climbRightConfig.softLimit
        // .forwardSoftLimit(DeepClimbConstants.FORWORD_SOFTLIMIT)
        // .forwardSoftLimitEnabled(true)
        // .reverseSoftLimit(DeepClimbConstants.REVERSE_SOFTLIMIT)
        // .reverseSoftLimitEnabled(true);

        m_climbLeft.configure(m_climbLeftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_climbRight.configure(m_climbRightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void setPosition(double position) {
        m_leftClosedLoopController.setReference(position, ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0);

    }

    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> this.setPosition(position));
    }

    public void run(double speed) {
        m_climbLeft.set(speed);
        m_climbRight.set(speed);
    }
    public void release(double speed){
        m_climbLeft.set(speed);
        m_climbRight.set(speed);
        m_climb21.set(speed);

    }

    public void run21(double speed) {
        m_climb21.set(speed);

    }

    public double getPosition(){
        return m_climb21.getEncoder().getPosition();
    }
    
    public boolean checkPositionOut(){
        if(m_climb21.getEncoder().getPosition() <= -
        5){
            return true; }
        else return false; }

    public boolean checkPositionIn(){
        if(m_climb21.getEncoder().getPosition() >= -.65){
            return true; }
        else return false;
    }              
    
    public BooleanSupplier stopClimbIn(){
        return(() -> checkPositionIn());

    }

    public BooleanSupplier stopClimbOut(){
        return(() -> checkPositionOut());
    }

   


@Override 
    public void periodic(){
        
        getPosition();

        checkPositionIn();
        checkPositionOut();

    



        SmartDashboard.putNumber("Climb Position ", m_climb21.getEncoder().getPosition());

        SmartDashboard.putBoolean("Climb out", checkPositionOut());
        SmartDashboard.putBoolean("Climb in", checkPositionIn());

        


    }
}