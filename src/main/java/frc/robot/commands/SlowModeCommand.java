// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveSpeedConstants;


public class SlowModeCommand extends Command {
    private Consumer<Double> setSpeed;
    private Consumer<Double> setAngularRate;
    private double defaultSpeed;
    private double defaultAngularRate;

    public SlowModeCommand(Consumer<Double> setSpeed, Consumer<Double> setAngularRate, double defaultSpeed, double defaultAngularRate) {
        this.setSpeed = setSpeed;
        this.setAngularRate = setAngularRate;
        this.defaultSpeed = defaultSpeed;
        this.defaultAngularRate = defaultAngularRate;
    }

    @Override
    public void initialize() {
        this.setSpeed.accept(defaultSpeed / DriveSpeedConstants.SLOW_SPEED_DIVISOR);
        this.setAngularRate.accept(defaultAngularRate / DriveSpeedConstants.SLOW_SPEED_DIVISOR);
    }

    @Override
    public void end(boolean interrupted) {
        this.setSpeed.accept(defaultSpeed);
        this.setAngularRate.accept(defaultAngularRate);
    }
}
