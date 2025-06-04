// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.Command;


public class SlowOnL4 extends Command {
    private Consumer<Double> slowSpeed;
    private Consumer<Double> setAngularRate;
    private double defaultSpeed;
    private double defaultAngularRate;
    private double elevatorLevel;
    private double slowSpeedMultiplyer;

    public SlowOnL4(double defaultSpeed, double defaultAngularRate, double elevatorLevel) {
        this.elevatorLevel = elevatorLevel;
        this.defaultSpeed = defaultSpeed;
        this.defaultAngularRate = defaultAngularRate;
    }

    @Override
    public void initialize() {
        if (elevatorLevel > 10) {
            slowSpeedMultiplyer = (8 - (elevatorLevel - 10)) / 8;
            this.slowSpeed.accept(defaultSpeed * slowSpeedMultiplyer);
            this.setAngularRate.accept(defaultAngularRate * slowSpeedMultiplyer);
        } else {
            this.slowSpeed.accept(defaultSpeed);
            this.setAngularRate.accept(defaultAngularRate);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        this.slowSpeed.accept(defaultSpeed);
        this.setAngularRate.accept(defaultAngularRate);
    }
}
