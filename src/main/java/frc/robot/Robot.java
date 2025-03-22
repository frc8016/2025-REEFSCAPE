// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private Vision m_Vision;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        // m_Vision = new Vision();
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // if (USE_VISION) {
        // // Correct pose estimate with vision measurements
        // var visionEst = m_Vision.getEstimatedGlobalPose();
        // visionEst.ifPresent(
        // est -> {
        // // Change our trust in the measurement based on the tags we can see
        // var estStdDevs = m_Vision.getEstimationStdDevs();
        // m_robotContainer.m_Drivetrain.addVisionMeasurement(
        // est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        // });
        // }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
