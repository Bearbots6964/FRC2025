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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagPositions;
import frc.robot.Robot;
import frc.robot.commands.PathfindingFactories.CoralStationSide;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class DriveToSpecificCoralStationCommand extends Command {

  private final Drive drive;
  private final Arm arm;
  private final Elevator elevator;
  private Command fullPath;
  private CoralStationSide side;

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToSpecificCoralStationCommand(
      Drive drive, CoralStationSide side, Arm arm, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.arm = arm;
    this.elevator = elevator;
    this.side = side;

    addRequirements(drive, arm, elevator);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Pose2d closestAprilTagPose = getSpecificCoralStationPose();
    Command pathfindPath =
        AutoBuilder.pathfindToPose(
            translateCoordinates(
                    closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5)
                .transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI))),
            new PathConstraints(
                1.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath pathToFront =
          new PathPlannerPath(
              PathPlannerPath.waypointsFromPoses(
                  translateCoordinates(
                      closestAprilTagPose,
                      closestAprilTagPose.getRotation().getDegrees() + 180,
                      0.5),
                  closestAprilTagPose),
              new PathConstraints(0.375, 1.0, 2 * Math.PI, 4 * Math.PI),
              null,
              new GoalEndState(
                  0.0, closestAprilTagPose.getRotation().rotateBy(new Rotation2d(Math.PI))));
      pathToFront.preventFlipping = true;
      fullPath = pathfindPath.andThen(AutoBuilder.followPath(pathToFront));
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

  private Pose2d getSpecificCoralStationPose() {
    Pose2d tagPose;
    if (Robot.getAlliance() == Alliance.Red) {
      tagPose =
          (side == CoralStationSide.LEFT)
              ? AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(1)
              : AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(2);
    } else {
      tagPose =
          (side == CoralStationSide.LEFT)
              ? AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(13)
              : AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(12);
    }

    return translateCoordinates(
        tagPose, tagPose.getRotation().getDegrees(), -Units.inchesToMeters(16.773));
  }

  private Pose2d translateCoordinates(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }
}
