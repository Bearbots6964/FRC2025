package frc.robot.automation.drivebase.requests

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import frc.robot.AprilTagPositions
import frc.robot.automation.drivebase.CoralStation
import frc.robot.automation.drivebase.CoralStationNudgeDirection
import frc.robot.automation.drivebase.CoralStationPositioning

/**
 * Represents a request for the robot to move to a specific coral station (LEFT or RIGHT) on the field.
 *
 * This class determines the target AprilTag, computes a transformation (including optional "nudge" adjustments),
 * and calculates the final pose the robot should reach. It is used to automate precise positioning at coral stations,
 * such as for game piece handling or scoring.
 *
 * @property coralStation The target coral station (LEFT or RIGHT).
 * @property coralStationNudgeDirection Optional nudge direction to slightly adjust the robot's position for better alignment or spacing.
 */
class CoralStationRequest(
    val coralStation: CoralStation,
    val coralStationNudgeDirection: CoralStationNudgeDirection = CoralStationNudgeDirection.NONE
) : AnchoredPointRequest {
    /**
     * The AprilTag ID associated with the selected coral station.
     */
    override val aprilTag: Int = getFieldLayout()
        .let {
            when (coralStation) {
                CoralStation.LEFT -> it.LEFT_CORAL_STATION
                CoralStation.RIGHT -> it.RIGHT_CORAL_STATION
            }
        }

    /**
     * The transformation to apply to the AprilTag pose to get the desired robot position.
     * This includes a fixed perpendicular offset and an optional nudge (inward or outward).
     */
    override val transformation: Transform2d = run {
        // First perform the nudge transformation based on the coral station and nudge direction.
        val nudgeDistance = when (coralStationNudgeDirection) {
            CoralStationNudgeDirection.INWARD -> CoralStationPositioning.NUDGE_DISTANCE_FROM_CORAL_STATION.let {
                if (coralStation == CoralStation.LEFT) -it else it
            }

            CoralStationNudgeDirection.OUTWARD -> CoralStationPositioning.NUDGE_DISTANCE_FROM_CORAL_STATION.let {
                if (coralStation == CoralStation.LEFT) it else -it
            }

            else -> 0.0
        }.let(Units::inchesToMeters) // Convert inches to meters using a scope function
        return@run Transform2d(
            -Units.inchesToMeters(CoralStationPositioning.PERPENDICULAR_DISTANCE_FROM_CORAL_STATION),
            nudgeDistance,
            Rotation2d()
        )
    }

    /**
     * The final pose (position and orientation) the robot should reach, calculated by applying
     * the transformation to the AprilTag's known field position.
     */
    override val pose: Pose2d = run {
        // Get the pose of the AprilTag and apply the transformation to it.
        val aprilTagPose =
            AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[aprilTag]
                ?: throw IllegalArgumentException("AprilTag with ID $aprilTag not found in the welded AprilTag positions.")
        return@run aprilTagPose.transformBy(transformation)
    }
}

