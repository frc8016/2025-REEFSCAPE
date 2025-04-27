// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import static frc.robot.Constants.PathfindToScoreConstants.*;

import org.json.simple.parser.ParseException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.PathfindToScoreConstants.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class PathfindToScore {
    CommandSwerveDrivetrain swerveDrivetrain;
    Direction direction;

    Supplier<Command> pathfindToScoreCommandSupplier = () -> {
        String pathName = constructPathName(getClosestPoseName(), direction);
        Optional<PathPlannerPath> maybePath = loadPath(pathName);

        
        if (maybePath.isPresent()) {
            return AutoBuilder.pathfindThenFollowPath(
                maybePath.get(),
                constraints
            );
        } else {
            return Commands.none();
        }
    };

    public PathfindToScore(CommandSwerveDrivetrain swerveDrivetrainIn, Direction directionIn) {
        swerveDrivetrain = swerveDrivetrainIn;
        direction = directionIn;
    }

    public Command createPathfindToScoreCommand() {
        return new DeferredCommand(pathfindToScoreCommandSupplier, Set.of(swerveDrivetrain));
    }

    private String getClosestPoseName() {
        Pose2d pose = swerveDrivetrain.getState().Pose.nearest(new ArrayList<>(getLineupPoseMap().keySet()));
        return getLineupPoseMap().get(pose);
    }

    private Map<Pose2d, String> getLineupPoseMap() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return RED_SCORING_POSITIONS;
        } else {
            return BLUE_SCORING_POSITIONS;
        }
    }

    private Optional<PathPlannerPath> loadPath(String pathName) {
        try {
            return Optional.of(PathPlannerPath.fromPathFile(pathName));
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("error in loading path");
            e.printStackTrace();
            return Optional.empty();
        }
    }

    private String constructPathName(String pointName, Direction direction) {
        String directionString = direction == Direction.LEFT ? "L" : "R";
        return "pathfind_to_" + pointName + "_" + directionString;
    }
}
