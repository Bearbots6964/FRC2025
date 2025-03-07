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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AprilTagPositions;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class DriveToNearestReefSideCommand {

  private Command fullPath;
  private final Drive drive;
  private boolean isLeftBumper = false;

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToNearestReefSideCommand(Drive drive, boolean isLeftBumper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.isLeftBumper = isLeftBumper;
  }

  // Called when the command is initially scheduled.
  public Command getCommand() {
    Pose2d closestAprilTagPose = getClosestReefAprilTagPose();
    Command pathfindPath =
        AutoBuilder.pathfindToPose(
            translateCoordinates(
                closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
            new PathConstraints(
                1.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath pathToFront =
          new PathPlannerPath(
              PathPlannerPath.waypointsFromPoses(
                  translateCoordinates(
                      closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
                  closestAprilTagPose),
              new PathConstraints(0.25, 1.0, 2 * Math.PI, 4 * Math.PI),
              null,
              new GoalEndState(0.0, closestAprilTagPose.getRotation()));
      pathToFront.preventFlipping = true;
      fullPath = pathfindPath.andThen(AutoBuilder.followPath(pathToFront));
      return fullPath;
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo =
        AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
      }
    }

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

    Pose2d inFrontOfAprilTag =
        translateCoordinates(
            closestPose, closestPose.getRotation().getDegrees(), -Units.inchesToMeters(23.773));

    Pose2d leftOrRightOfAprilTag;
    if (isLeftBumper) {
      leftOrRightOfAprilTag =
          translateCoordinates(
              inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1432265);
    } else {
      leftOrRightOfAprilTag =
          translateCoordinates(
              inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1432265);
    }

    if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
      if (isLeftBumper) {
        leftOrRightOfAprilTag =
            translateCoordinates(
                inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1432265);
      } else {
        leftOrRightOfAprilTag =
            translateCoordinates(
                inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1432265);
      }
    }

    return leftOrRightOfAprilTag;
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
