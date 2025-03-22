package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.AprilTagPositions
import frc.robot.Constants.PathfindingConstants
import frc.robot.Robot
import frc.robot.Robot.Companion.alliance
import frc.robot.commands.PathfindingFactories.Reef.*
import frc.robot.subsystems.drive.Drive
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

object PathfindingFactories {
    //<editor-fold desc="Coral Station Pathfinding">
    // Nearest pathfinding
    fun pathfindToNearestCoralStation(drive: Drive): Command = AutoBuilder.pathfindToPose(
        getClosestCoralStationAprilTagPose(drive.pose).let {
            translateCoordinates(
                it, it.rotation.degrees, -PathfindingConstants.pathfindingEndDistanceFromGoal
            )
        }, PathfindingConstants.pathfindingConstraints, 0.0
    )

    fun finalLineupToNearestCoralStation(drive: Drive): Command {
        val currentPose = drive.pose
        val targetPose = getClosestCoralStationAprilTagPose(currentPose)
        val path = PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                listOf(
                    currentPose, targetPose
                )
            ),
            PathfindingConstants.finalLineupConstraints,
            null,
            GoalEndState(0.0, targetPose.rotation)
        )
        path.preventFlipping = true
        return AutoBuilder.followPath(path)
    }

    // Specific pathfinding
    fun pathfindToSpecificCoralStation(drive: Drive, side: CoralStationSide): Command =
        AutoBuilder.pathfindToPose(
            getSpecificCoralStationPose(side), PathfindingConstants.pathfindingConstraints, 0.0
        )

    fun finalLineupToSpecificCoralStation(drive: Drive, side: CoralStationSide): Command {
        val currentPose = drive.pose
        val targetPose = getSpecificCoralStationPose(side)
        val path = PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                listOf(
                    currentPose, targetPose
                )
            ),
            PathfindingConstants.finalLineupConstraints,
            null,
            GoalEndState(0.0, targetPose.rotation)
        )
        path.preventFlipping = true
        return AutoBuilder.followPath(path)
    }

    // Alternate pathfinding
    fun pathfindToCoralStationAlternate(
        drive: Drive, side: CoralStationSide, nudge: () -> Translation2d
    ): Command = drive.followRepulsorField(getSpecificCoralStationPose(side), nudge)
    //</editor-fold>

    // <editor-fold desc="Reef Pathfinding">
    fun pathfindToNearestReef(drive: Drive, side: ReefSides): Command {
        val currentPose = drive.pose
        val targetPose = getClosestReefAprilTagPose(currentPose, side)
        return AutoBuilder.pathfindToPose(
            targetPose, PathfindingConstants.pathfindingConstraints, 0.0
        )
    }

    fun pathfindToReefAlternate(drive: Drive, reef: Reef, nudge: () -> Translation2d): Command {
        val currentPose = drive.pose
        val targetPose = getSpecificReefSidePose(reef)
        return drive.followRepulsorField(targetPose, nudge)
    }

    fun finalLineupToNearestReef(drive: Drive, side: ReefSides): Command {
        val currentPose = drive.pose
        val targetPose = getClosestReefAprilTagPose(currentPose, side)
        val path = PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                listOf(
                    currentPose, targetPose
                )
            ),
            PathfindingConstants.finalLineupConstraints,
            null,
            GoalEndState(0.0, targetPose.rotation)
        )
        path.preventFlipping = true
        return AutoBuilder.followPath(path)
    }

    fun pathfindToSpecificReef(drive: Drive, reef: Reef): Command {
        val currentPose = drive.pose
        val targetPose = getSpecificReefSidePose(reef)
        return AutoBuilder.pathfindToPose(
            targetPose, PathfindingConstants.pathfindingConstraints, 0.0
        )
    }

    fun finalLineupToSpecificReef(drive: Drive, reef: Reef): Command {
        val currentPose = drive.pose
        val targetPose = getSpecificReefSidePose(reef)
        val path = PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                listOf(
                    currentPose, targetPose
                )
            ),
            PathfindingConstants.finalLineupConstraints,
            null,
            GoalEndState(0.0, targetPose.rotation)
        )
        path.preventFlipping = true
        return AutoBuilder.followPath(path)
    }

    // </editor-fold>

    // <editor-fold desc="Utility Methods">
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

    private fun getClosestReefAprilTagPose(currentPose: Pose2d, side: ReefSides): Pose2d {
        var aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS
        if (alliance == Alliance.Red) {
            aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS
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

        val inFrontOfAprilTag: Pose2d = translateCoordinates(
            closestPose,
            closestPose.rotation.degrees,
            -PathfindingConstants.finalDistanceFromReefMeters
        )
        val leftOrRightOfAprilTag: Pose2d = when (side) {
            ReefSides.LEFT -> {
                translateCoordinates(
                    inFrontOfAprilTag,
                    closestPose.rotation.degrees + 90,
                    PathfindingConstants.lateralDistanceFromReefMeters
                )
            }

            ReefSides.RIGHT -> {
                translateCoordinates(
                    inFrontOfAprilTag,
                    closestPose.rotation.degrees + 90,
                    -PathfindingConstants.lateralDistanceFromReefMeters
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
            if (side == CoralStationSide.LEFT) AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[1]
            else AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[2]
        } else {
            if (side == CoralStationSide.LEFT) AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[13]
            else AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[12]
        }

        return translateCoordinates(
            tagPose!!.transformBy(Transform2d(0.0, 0.0, Rotation2d.fromDegrees(180.0))),
            tagPose.rotation.degrees,
            -PathfindingConstants.finalDistanceFromCoralStationMeters
        )
    }

    private fun getSpecificReefSidePose(reef: Reef): Pose2d {
        val aprilTagsToAlignTo = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS
        val alliance = Robot.alliance
        val aprilTagNum: Int = when (reef) {
            A, B, AB_ALGAE -> when (alliance) {
                Alliance.Red -> 7
                Alliance.Blue -> 18
            }

            C, D, CD_ALGAE -> when (alliance) {
                Alliance.Red -> 8
                Alliance.Blue -> 17
            }

            E, F, EF_ALGAE -> when (alliance) {
                Alliance.Red -> 9
                Alliance.Blue -> 22
            }

            G, H, GH_ALGAE -> when (alliance) {
                Alliance.Red -> 10
                Alliance.Blue -> 21
            }

            I, J, IJ_ALGAE -> when (alliance) {
                Alliance.Red -> 11
                Alliance.Blue -> 20
            }

            K, L, KL_ALGAE -> when (alliance) {
                Alliance.Red -> 6
                Alliance.Blue -> 19
            }
        }
        val side = when (reef) {
            A, C, E, G, I, K -> ReefSides.LEFT
            B, D, F, H, J, L -> ReefSides.RIGHT
            AB_ALGAE, CD_ALGAE, EF_ALGAE, GH_ALGAE, IJ_ALGAE, KL_ALGAE -> ReefSides.MIDDLE
        }

        val closestPose = aprilTagsToAlignTo[aprilTagNum]

        val inFrontOfAprilTag: Pose2d = translateCoordinates(
            closestPose!!,
            closestPose.rotation.degrees,
            -PathfindingConstants.finalDistanceFromCoralStationMeters
        )
        val leftOrRightOfAprilTag: Pose2d = when (side) {
            ReefSides.LEFT -> {
                translateCoordinates(
                    inFrontOfAprilTag,
                    closestPose.rotation.degrees + 90,
                    PathfindingConstants.lateralDistanceFromReefMeters
                )
            }

            ReefSides.RIGHT -> {
                translateCoordinates(
                    inFrontOfAprilTag,
                    closestPose.rotation.degrees + 90,
                    -PathfindingConstants.lateralDistanceFromReefMeters
                )
            }

            else -> inFrontOfAprilTag
        }


        return leftOrRightOfAprilTag
    }

    enum class CoralStationSide {
        LEFT, RIGHT
    }

    enum class Reef {
        A, B, C, D, E, F, G, H, I, J, K, L, AB_ALGAE, CD_ALGAE, EF_ALGAE, GH_ALGAE, IJ_ALGAE, KL_ALGAE
    }

    enum class ReefSides {
        LEFT, RIGHT, MIDDLE
    }

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
    // </editor-fold>
}