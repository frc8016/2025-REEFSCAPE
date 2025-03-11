// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.SetPointConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                  // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity
    // create contollers
    private final CommandXboxController m_swerveController = new CommandXboxController(0);// swerve xbox controller
    private final CommandXboxController m_operatorController = new CommandXboxController(1); // elevator/climb etc
                                                                                             // xbox
                                                                                             // controller
    // create instnace of subsystems
    public final CoralIntake m_CoralIntake = new CoralIntake();
    public final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    // private final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final Elevator m_Elevator = new Elevator();
    private final DeepClimb m_DeepClimb = new DeepClimb();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        NamedCommands.registerCommand("Elevator",
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL4));

        configureBindings();
    }

    private void configureBindings() {
        // Swerve Code
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_Drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                m_Drivetrain.applyRequest(() -> drive
                        .withVelocityX(m_swerveController.getLeftY() * MaxSpeed) // Drive
                                                                                 // forward
                                                                                 // with
                                                                                 // negative
                                                                                 // Y
                                                                                 // (forward)
                        .withVelocityY(m_swerveController.getLeftX() * MaxSpeed) // Drive left
                                                                                 // with
                                                                                 // negative X
                                                                                 // (left)
                        .withRotationalRate(-m_swerveController.getRightX() * MaxAngularRate) // Drive
                                                                                              // counterclockwise
                                                                                              // with
                                                                                              // negative
                                                                                              // X
                                                                                              // (left)
                ));

        m_swerveController.a().whileTrue(m_Drivetrain.applyRequest(() -> brake));
        m_swerveController.b().whileTrue(m_Drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-m_swerveController.getLeftY(),
                        -m_swerveController.getLeftX()))));

        m_swerveController.pov(0)
                .whileTrue(m_Drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        m_swerveController.pov(180)
                .whileTrue(m_Drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        m_swerveController.x().whileTrue(
                new StartEndCommand(() -> m_DeepClimb.runLeft(.5),
                        () -> m_DeepClimb.runLeft(0), m_DeepClimb));
        m_swerveController.y().whileTrue(
                new StartEndCommand(() -> m_DeepClimb.runRight(.5),
                        () -> m_DeepClimb.runRight(0), m_DeepClimb));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_swerveController.back().and(m_swerveController.y())
        // .whileTrue(m_Drivetrain.sysIdDynamic(Direction.kForward));
        // m_swerveController.back().and(m_swerveController.x())
        // .whileTrue(m_Drivetrain.sysIdDynamic(Direction.kReverse));
        // m_swerveController.start().and(m_swerveController.y())
        // .whileTrue(m_Drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_swerveController.start().and(m_swerveController.x())
        // .whileTrue(m_Drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_swerveController.leftBumper().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldCentric()));

        m_Drivetrain.registerTelemetry(logger::telemeterize);

        // Coral outtake code
        // RUns multiple commands in order
        m_operatorController.leftBumper().toggleOnTrue(
                new SequentialCommandGroup(
                        // runs rollers at .1 speed until the beam is broken
                        new StartEndCommand(
                                () -> m_CoralIntake.runRollers(.1),
                                () -> m_CoralIntake.runRollers(.1),
                                m_CoralIntake)
                                .until(m_CoralIntake.isBroken()),
                        // runs rollers at .07 speed until beak is restored
                        new StartEndCommand(
                                () -> m_CoralIntake.runRollers(.07),
                                () -> m_CoralIntake.runRollers(0),
                                m_CoralIntake)
                                .until(m_CoralIntake.notBroken())));

        // change to start end commands
        // m_operatorController.leftBumper().whileTrue(
        // new StartEndCommand(
        // () -> m_CoralIntake.runRollers(0.1),
        // () -> m_CoralIntake.runRollers(0),
        // m_CoralIntake));

        m_operatorController.rightBumper().whileTrue(
                new StartEndCommand(
                        () -> m_CoralIntake.runRollers(0.2),
                        () -> m_CoralIntake.runRollers(0),
                        m_CoralIntake));

        m_operatorController.leftTrigger().whileTrue(
                new StartEndCommand(
                        () -> m_CoralIntake.runRollers(-0.1),
                        () -> m_CoralIntake.runRollers(0),
                        m_CoralIntake));

        m_operatorController.y().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL4));

        m_operatorController.b().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL3));

        m_operatorController.x().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL2));

        m_operatorController.a().onTrue(
                m_Elevator.goToSetPointCommand(SetPointConstants.LEVEL1));

        // m_operatorController.rightTrigger().onTrue(
        // m_AlgaeIntake.goToSetPointCommand(AlgaeIntakeConstants.UP_POSITION));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
