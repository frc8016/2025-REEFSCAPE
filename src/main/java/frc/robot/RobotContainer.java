// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.SetPointConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top                                                                         // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second mx angular velocity 
                                                                                     
    // create contollers
    private final CommandXboxController m_swerveController = new CommandXboxController(0);// swerve/deep climb xbox controller
    private final CommandXboxController m_operatorController = new CommandXboxController(1); // elevator/intake/outake contoller 
                                                                                             
    // create instnace of subsystems
    public final CoralIntake m_CoralIntake = new CoralIntake();
    public final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.createDrivetrain();
    public final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
    private final Elevator m_Elevator = new Elevator();
    private final DeepClimb m_DeepClimb = new DeepClimb();
    private final Funnel m_Funnel = new Funnel();
    private final SendableChooser<Command> autoChooser;
    private final Telemetry logger = new Telemetry(MaxSpeed);


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
                                                                    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    @SuppressWarnings("unused")
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public RobotContainer() {

        NamedCommands.registerCommand("elevatorLevel4",
                m_Elevator.goToSetPointWithWaitCommand(SetPointConstants.LEVEL4));
        NamedCommands.registerCommand("elevatorLevel2",
                m_Elevator.goToSetPointWithWaitCommand(SetPointConstants.LEVEL2));
        NamedCommands.registerCommand("elevatorLevel1",
                m_Elevator.goToSetPointWithWaitCommand(SetPointConstants.LEVEL1));
        NamedCommands.registerCommand("intakeCoral",
                m_CoralIntake.intakeCommand());
        NamedCommands.registerCommand("outtakeCoral",
                m_CoralIntake.outCommand().withTimeout(1));
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Swerve Code
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_Drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                m_Drivetrain.applyRequest(() -> drive
                        .withVelocityX(m_swerveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(m_swerveController.getLeftX() * MaxSpeed) // Drive leftwith negative X (left)
                        .withRotationalRate(-m_swerveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        m_swerveController.a().whileTrue(m_Drivetrain.applyRequest(() -> brake));

        m_swerveController.b().whileTrue(m_Drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-m_swerveController.getLeftY(),
                        -m_swerveController.getLeftX()))));

        // reset the field-centric heading on left bumper press
        m_swerveController.leftBumper().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldCentric()));
/*Deep climb code */
        m_swerveController.rightTrigger().whileTrue(
                new StartEndCommand(
                       () -> m_DeepClimb.run(.5), 
                       () -> m_DeepClimb.run(0), 
                                m_DeepClimb).until(m_DeepClimb.stopClimbIn()));

        m_swerveController.leftTrigger().whileTrue(
                new StartEndCommand(
                        () -> m_DeepClimb.release(-.3), 
                        () -> m_DeepClimb.release(0), 
                        m_DeepClimb).until(m_DeepClimb.stopClimbOut()));

        m_swerveController.rightBumper().onTrue(
                new StartEndCommand(
                       () -> m_Funnel.run(-.1), 
                       () -> m_Funnel.run(0), 
                       m_Funnel).until(m_Funnel.triggered())
        );
       

       

        m_Drivetrain.registerTelemetry(logger::telemeterize);
/*Operator Controls */

        m_operatorController.leftBumper().toggleOnTrue(
                m_CoralIntake.intakeCommand());

        m_operatorController.rightBumper().whileTrue(
                m_CoralIntake.outCommand());

        m_operatorController.y().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL4));

        m_operatorController.b().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL3));

        m_operatorController.x().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL2));

        m_operatorController.a().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL1));
        m_operatorController.povUp().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.TROUGH));
        
        

        m_operatorController.rightTrigger().whileTrue(
                new StartEndCommand(
                        () -> m_AlgaeIntake.setPosition(AlgaeIntakeConstants.DOWN_POSITION),
                        () -> m_AlgaeIntake.setPosition(AlgaeIntakeConstants.UP_POSITION),
                        m_AlgaeIntake));
    }


    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void setMaxSpeed(double speed) {
        this.MaxSpeed = speed;
    }

    public void setMaxAngularRate(double angularRate) {
        this.MaxAngularRate = angularRate;
    }
}
