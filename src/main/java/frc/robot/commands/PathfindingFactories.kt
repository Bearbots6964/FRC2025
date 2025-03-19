package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.AprilTagPositions
import frc.robot.Constants.PathfindingConstants
import frc.robot.Robot.Companion.alliance
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

object PathfindingFactories {

    private fun translateCoordinates(
        originalPose: Pose2d, degreesRotate: Double, distance: Double
    ): Pose2d {
        val newXCoord = originalPose.x + (cos(Math.toRadians(degreesRotate)) * distance)
        val newYCoord = originalPose.y + (sin(Math.toRadians(degreesRotate)) * distance)

        return Pose2d(newXCoord, newYCoord, originalPose.rotation)
    }

    private fun findDistanceBetween(pose1: Pose2d, pose2: Pose2d): Double {
        return hypot(pose1.x - pose2.x, pose1.y - pose2.y)
    }
    private fun getClosestCoralStationAprilTagPose(currentPose: Pose2d): Pose2d {
        val aprilTagsToAlignTo = HashMap<Int, Pose2d>()
        if (alliance == Alliance.Red) {
            aprilTagsToAlignTo[1] = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[1]!!
            aprilTagsToAlignTo[2] = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[2]!!
        } else {
            aprilTagsToAlignTo[12] = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[12]!!
            aprilTagsToAlignTo[13] = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[13]!!
        }

        var closestPose = Pose2d()
        var closestDistance = Double.MAX_VALUE

        for ((_, pose) in aprilTagsToAlignTo) {
            val distance = findDistanceBetween(currentPose, pose)
            if (distance < closestDistance) {
                closestDistance = distance
                closestPose = pose
            }
        }

        return translateCoordinates(
            closestPose,
            closestPose.rotation.degrees,
            -PathfindingConstants.finalDistanceFromCoralStationMeters
        )
    }
    private enum class ReefSides {
        LEFT, RIGHT, MIDDLE
    }

    private fun getClosestReefAprilTagPose(currentPose: Pose2d, side: ReefSides): Pose2d {
        var aprilTagsToAlignTo =
            AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS
        if (alliance == Alliance.Red) {
            aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS
        }

        var closestPose = Pose2d()
        var closestDistance = Double.MAX_VALUE
        -1

        for ((key, pose) in aprilTagsToAlignTo) {
            val distance = findDistanceBetween(currentPose, pose)
            if (distance < closestDistance) {
                closestDistance = distance
                closestPose = pose
            }
        }

        val inFrontOfAprilTag: Pose2d =
            translateCoordinates(
                closestPose, closestPose.rotation.degrees, -PathfindingConstants.finalDistanceFromReefMeters
            )

        var leftOrRightOfAprilTag: Pose2d
        leftOrRightOfAprilTag = when (side) {
            ReefSides.LEFT -> {
                translateCoordinates(inFrontOfAprilTag, closestPose.rotation.degrees + 90, PathfindingConstants.lateralDistanceFromReefMeters)
            }
            ReefSides.RIGHT -> {
                translateCoordinates(
                    inFrontOfAprilTag, closestPose.rotation.degrees + 90, -PathfindingConstants.lateralDistanceFromReefMeters
                )
            }
            else -> {
                inFrontOfAprilTag
            }
        }

        return leftOrRightOfAprilTag
    }
    private fun getSpecificCoralStationPose(side: CoralStationSide): Pose2d {
        val tagPose = if (alliance == Alliance.Red) {
            if (side == CoralStationSide.LEFT)
                AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[1]
            else
                AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[2]
        } else {
            if (side == CoralStationSide.LEFT)
                AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[13]
            else
                AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[12]
        }

        return translateCoordinates(
            tagPose!!, tagPose.rotation.degrees, -PathfindingConstants.finalDistanceFromCoralStationMeters
        )
    }
    enum class CoralStationSide {
        LEFT,
        RIGHT
    }
}