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
import frc.robot.Constants.SuperstructureConstants.SuperstructureState;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import java.util.List;
import java.util.function.Supplier;

public class DriveToSpecificReefSideCommand extends Command {

  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Supplier<SuperstructureState> elevatorState;
  private Command fullPath;
  private int[] numbers;
  private boolean isLeftSide;

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToSpecificReefSideCommand(
      Drive drive,
      Elevator elevator,
      Arm arm,
      Supplier<SuperstructureState> elevatorState,
      Reef reef) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.elevator = elevator;
    this.arm = arm;
    this.elevatorState = elevatorState;
    addRequirements(drive);
    numbers =
        switch (reef) {
          case A, B, AB_ALGAE -> new int[] {7, 18};
          case C, D, CD_ALGAE -> new int[] {8, 17};
          case E, F, EF_ALGAE -> new int[] {9, 22};
          case G, H, GH_ALGAE -> new int[] {10, 21};
          case I, J, IJ_ALGAE -> new int[] {11, 20};
          case K, L, KL_ALGAE -> new int[] {6, 19};
        };
    isLeftSide =
        switch (reef) {
          case A, C, E, G, I, K -> true;
          case B, D, F, H, J, L, KL_ALGAE, IJ_ALGAE, GH_ALGAE, EF_ALGAE, CD_ALGAE, AB_ALGAE ->
              false;
        };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var closestAprilTagPose = getSpecificReefSidePose();
    var pathfindPath =
        AutoBuilder.pathfindToPose(
            translateCoord(
                closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
            new PathConstraints(
                3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));

    try {
      // Load the path you want to follow using its name in the GUI
      var pathToFront =
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
          SuperstructureCommands.INSTANCE
              .goToPosition(elevator, arm, SuperstructureState.HOME)
              .alongWith(pathfindPath)
              .andThen(
                  SuperstructureCommands.INSTANCE
                      .goToPosition(elevator, arm, elevatorState.get())
                      .alongWith(AutoBuilder.followPath(pathToFront)));
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

  private Pose2d getSpecificReefSidePose() {
    var aprilTagsToAlignTo = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS;
    int aprilTagNum =
        switch (Robot.getAlliance()) {
          case Red -> numbers[0];
          case Blue -> numbers[1];
        };

    var closestPose = aprilTagsToAlignTo.get(aprilTagNum);

    var inFrontOfAprilTag =
        translateCoord(
            closestPose, closestPose.getRotation().getDegrees(), -Units.inchesToMeters(18.773));

    Pose2d leftOrRightOfAprilTag;
    if (isLeftSide) {
      leftOrRightOfAprilTag =
          translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1686306);
    } else {
      leftOrRightOfAprilTag =
          translateCoord(
              inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1686306);
    }

    if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
      if (isLeftSide) {
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

  public enum Reef {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    AB_ALGAE,
    CD_ALGAE,
    EF_ALGAE,
    GH_ALGAE,
    IJ_ALGAE,
    KL_ALGAE
  }
}
