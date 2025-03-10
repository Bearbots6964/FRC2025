// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thank you (and sorry) to team 7414 for this code
// This file was copied from another team's repository and modified to fit our needs
package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.AprilTagPositions
import frc.robot.subsystems.drive.Drive
import lombok.Getter
import org.littletonrobotics.junction.AutoLogOutput
import java.util.function.BooleanSupplier
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class ReefPathfinding(private val drive: Drive, private val isLeftBumper: BooleanSupplier) {
    @Getter
     val pathfindPath: Command

    @Getter
     var pathToFront: Command? = null

    val fullPath: Command
        get() {
            if (pathToFront == null) {
                return pathfindPath
            }
            return pathfindPath.andThen(pathToFront)
        }

    /**
     * Creates a new DriveToNearestReefSideCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.

        val closestAprilTagPose = closestReefAprilTagPose
        pathfindPath = AutoBuilder.pathfindToPose(
            translateCoord(
                closestAprilTagPose, closestAprilTagPose.rotation.degrees, -0.5
            ), PathConstraints(
                1.0, 1.0, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)
            )
        ).withName("Pathfind to Reef")

        try {
            // Load the path you want to follow using its name in the GUI
            val frontPath = PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                    translateCoord(
                        closestAprilTagPose, closestAprilTagPose.rotation.degrees, -0.5
                    ), closestAprilTagPose
                ),
                PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI),
                null,
                GoalEndState(0.0, closestAprilTagPose.rotation)
            )
            frontPath.preventFlipping = true
            pathToFront = AutoBuilder.followPath(frontPath).withName("Final Reef Lineup")
        } catch (e: Exception) {
            DriverStation.reportError("Big oops: " + e.message, e.stackTrace)
        }
    }

    @get:AutoLogOutput(key = "Auto/ClosestReefPose")
    private val closestReefAprilTagPose: Pose2d
        // Called when the command is initially scheduled.
        get() {
            var aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS
            val alliance = DriverStation.getAlliance()
            if (alliance.isPresent) {
                if (alliance.get() == Alliance.Red) {
                    aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS
                }
            }

            val currentPose = drive.pose
            var closestPose = Pose2d()
            var closestDistance = Double.MAX_VALUE
            var aprilTagNum = -1

            for ((key, pose) in aprilTagsToAlignTo) {
                val distance = findDistanceBetween(currentPose, pose)
                if (distance < closestDistance) {
                    closestDistance = distance
                    closestPose = pose
                    aprilTagNum = key
                }
            }

            val inFrontOfAprilTag = translateCoord(
                closestPose, closestPose.rotation.degrees, -Units.inchesToMeters(23.773)
            )

            var leftOrRightOfAprilTag: Pose2d
            leftOrRightOfAprilTag = if (isLeftBumper.asBoolean) {
                translateCoord(
                    inFrontOfAprilTag, closestPose.rotation.degrees + 90, 0.1432265
                )
            } else {
                translateCoord(
                    inFrontOfAprilTag, closestPose.rotation.degrees + 90, -0.1432265
                )
            }

            if (listOf(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
                leftOrRightOfAprilTag = if (isLeftBumper.asBoolean) {
                    translateCoord(
                        inFrontOfAprilTag, closestPose.rotation.degrees + 90, -0.1432265
                    )
                } else {
                    translateCoord(
                        inFrontOfAprilTag, closestPose.rotation.degrees + 90, 0.1432265
                    )
                }
            }

            return leftOrRightOfAprilTag
        }

    private fun translateCoord(
        originalPose: Pose2d, degreesRotate: Double, distance: Double
    ): Pose2d {
        val newXCoord = originalPose.x + (cos(Math.toRadians(degreesRotate)) * distance)
        val newYCoord = originalPose.y + (sin(Math.toRadians(degreesRotate)) * distance)

        return Pose2d(newXCoord, newYCoord, originalPose.rotation)
    }

    private fun findDistanceBetween(pose1: Pose2d, pose2: Pose2d): Double {
        return sqrt(
            (pose2.x - pose1.x).pow(2.0) + (pose2.y - pose1.y).pow(2.0)
        )
    }
}
