// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Thank you (and sorry) to team 7414 for this code

// This file was copied from another team's repository and modified to fit our needs
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagPositions;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashMap;
import java.util.Map;

public class DriveToNearestCoralStationCommand extends Command {

  private final Drive drive;
  private final Arm arm;
  private final Elevator elevator;
  private Command fullPath;

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToNearestCoralStationCommand(Drive drive, Arm arm, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.arm = arm;
    this.elevator = elevator;

    addRequirements(drive, arm, elevator);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    var closestAprilTagPose = getClosestCoralStationAprilTagPose();
    var pathfindPath =
        AutoBuilder.pathfindToPose(
            translateCoordinates(
                    closestAprilTagPose,
                    closestAprilTagPose.getRotation().getDegrees(),
                    -PathfindingConstants.pathfindingEndDistanceFromGoal)
                .transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI))),
            PathfindingConstants.getPathfindingConstraints());

    try {
      // Load the path you want to follow using its name in the GUI
      var pathToFront =
          new PathPlannerPath(
              PathPlannerPath.waypointsFromPoses(
                  translateCoordinates(
                      closestAprilTagPose,
                      closestAprilTagPose.getRotation().getDegrees() + 180,
                      PathfindingConstants.pathfindingEndDistanceFromGoal),
                  closestAprilTagPose),
              PathfindingConstants.getFinalLineupConstraints(),
              null,
              new GoalEndState(
                  0.0, closestAprilTagPose.getRotation().rotateBy(new Rotation2d(Math.PI))));
      pathToFront.preventFlipping = true;
      fullPath =
          pathfindPath.andThen(
              SuperstructureCommands.INSTANCE
                  .coralStationPosition(elevator, arm)
                  .alongWith(AutoBuilder.followPath(pathToFront)));
      fullPath.schedule();
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (fullPath != null && fullPath.isScheduled()) {
      fullPath.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return fullPath.isScheduled() && fullPath.isFinished();
  }

  private Pose2d getClosestCoralStationAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = new HashMap<>();
    if (Robot.getAlliance() == Alliance.Red) {
      aprilTagsToAlignTo.put(1, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(1));
      aprilTagsToAlignTo.put(2, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(2));
    } else {
      aprilTagsToAlignTo.put(12, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(12));
      aprilTagsToAlignTo.put(13, AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(13));
    }

    var currentPose = drive.getPose();
    var closestPose = new Pose2d();
    var closestDistance = Double.MAX_VALUE;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      var pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
      }
    }

    return translateCoordinates(
        closestPose,
        closestPose.getRotation().getDegrees(),
        -PathfindingConstants.finalDistanceFromCoralStationMeters);
  }

  private Pose2d translateCoordinates(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.hypot(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY());
  }
}
