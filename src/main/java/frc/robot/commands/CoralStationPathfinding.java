// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Thank you (and sorry) to team 7414 for this code

// This file was copied from another team's repository and modified to fit our needs
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagPositions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class CoralStationPathfinding {

  private final Drive drive;
  @Getter Command pathfindPath;
  @Getter Command pathToFront;
  private Pose2d closestAprilTagPose;

  public Command getFullPath() {
    return pathfindPath.andThen(pathToFront);
  }

  /** Creates a new DriveToNearestReefSideCommand. */
  public CoralStationPathfinding(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    RobotContainer.getStatusTopic().set("Driving to nearest coral station");
    closestAprilTagPose = getClosestCoralStationAprilTagPose();
    pathfindPath =
        AutoBuilder.pathfindToPose(
            translateCoordinates(
                    closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5)
                .transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI))),
            new PathConstraints(
                1.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
    PathPlannerPath frontPath =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                translateCoordinates(
                    closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees() + 180, 0.5),
                closestAprilTagPose),
            new PathConstraints(0.25, 1.0, 2 * Math.PI, 4 * Math.PI),
            null,
            new GoalEndState(
                0.0, closestAprilTagPose.getRotation().rotateBy(new Rotation2d(Math.PI))));
    frontPath.preventFlipping = true;
    pathToFront = AutoBuilder.followPath(frontPath);
  }

  // Called when the command is initially scheduled.

  @AutoLogOutput
  private Pose2d getClosestCoralStationAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = new HashMap<>();
    aprilTagsToAlignTo.put(1, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(1));
    aprilTagsToAlignTo.put(2, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(2));
    aprilTagsToAlignTo.put(12, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(12));
    aprilTagsToAlignTo.put(13, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(13));

    Pose2d currentPose = drive.getPose();
    Pose2d closestPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    Integer aprilTagNum = -1;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
        aprilTagNum = entry.getKey();
      }
    }

    return translateCoordinates(
        closestPose, closestPose.getRotation().getDegrees(), -Units.inchesToMeters(14.773));
  }

  private Pose2d translateCoordinates(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(
        Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}
