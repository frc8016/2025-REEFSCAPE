// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class PathfindToScore extends Command {

    PathPlannerPath path;
    public String pathName = "pathfind_to_C1";
    Command pathfindingCommand;

    public PathfindToScore () {
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("error in loading path");
            e.printStackTrace();
        }

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            path = path.flipPath();
        }
    }

    PathConstraints constraints = new PathConstraints(
        5.210, 7.1,
        Units.degreesToRadians(180), Units.degreesToRadians(-120));

    @Override
    public void initialize() {
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints);

        pathfindingCommand.initialize();
        System.out.println(this.path);
    }

    @Override
    public void schedule() {
        super.schedule();
        pathfindingCommand.schedule();
        System.out.println("COMMAND WAS SCHEDULED");
    }
}
