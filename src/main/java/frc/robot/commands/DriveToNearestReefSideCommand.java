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
import frc.robot.AprilTagPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveToNearestReefSideCommand extends Command {

  private Command fullPath;
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Supplier<ElevatorState> elevatorState;
  private boolean isLeftBumper = false;

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToNearestReefSideCommand(
      Drive drive,
      boolean isLeftBumper,
      Elevator elevator,
      Arm arm,
      Supplier<ElevatorState> elevatorState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.isLeftBumper = isLeftBumper;
    this.elevator = elevator;
    this.arm = arm;
    this.elevatorState = elevatorState;

    addRequirements(drive, elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d closestAprilTagPose = getClosestReefAprilTagPose();
    Command pathfindPath =
        AutoBuilder.pathfindToPose(
            translateCoord(
                closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
            new PathConstraints(
                3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath pathToFront =
          new PathPlannerPath(
              PathPlannerPath.waypointsFromPoses(
                  translateCoord(
                      closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
                  closestAprilTagPose),
              new PathConstraints(0.5, 1.0, 2 * Math.PI, 4 * Math.PI),
              null,
              new GoalEndState(0.0, closestAprilTagPose.getRotation()));
      pathToFront.preventFlipping = true;
      fullPath =
          ReefPositionCommands.INSTANCE
              .goToPosition(elevator, arm, ElevatorState.HOME)
              .alongWith(pathfindPath)
              .andThen(
                  ReefPositionCommands.INSTANCE.goToPosition(elevator, arm, elevatorState.get()))
              .andThen(AutoBuilder.followPath(pathToFront));
      fullPath.schedule();
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (fullPath != null) {
      fullPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo =
        AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
      if (Robot.getAlliance() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
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
        translateCoord(
            closestPose, closestPose.getRotation().getDegrees(), -Units.inchesToMeters(18.773));

    Pose2d leftOrRightOfAprilTag;
    if (isLeftBumper) {
      leftOrRightOfAprilTag =
          translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1686306);
    } else {
      leftOrRightOfAprilTag =
          translateCoord(
              inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1686306);
    }

    if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
      if (isLeftBumper) {
        leftOrRightOfAprilTag =
            translateCoord(
                inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1686306);
      } else {
        leftOrRightOfAprilTag =
            translateCoord(
                inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1686306);
      }
    }

    return leftOrRightOfAprilTag;
  }

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(
        Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}
