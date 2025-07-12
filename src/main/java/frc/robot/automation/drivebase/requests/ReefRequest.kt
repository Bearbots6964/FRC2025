package frc.robot.automation.drivebase.requests

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import frc.robot.AprilTagPositions
import frc.robot.automation.drivebase.CoralPlacement
import frc.robot.automation.drivebase.ReefLocation
import frc.robot.automation.drivebase.ReefPositioning
import frc.robot.automation.drivebase.leftPositions

/**
 * A request to drive to a specific location on the reef.
 * This class encapsulates the logic for determining the AprilTag ID,
 * the transformation to apply to the AprilTag pose,
 * and the final pose of the robot at that location.
 * * @property location The specific reef location to drive to, which can be a [CoralPlacement] or an [frc.robot.automation.drivebase.AlgaePlacement].
 */
class ReefRequest(val location: ReefLocation, val reefDistance: ReefDistance = ReefDistance.CLOSE) : AnchoredPointRequest {
    override val aprilTag: Int = run {
        FieldLayout.getTag(location)
    }
    override val transformation: Transform2d = run {
        val lateral: Double = if (location is CoralPlacement) {
            if (leftPositions.contains(location)) Units.inchesToMeters(ReefPositioning.LEFT_REEF_LATERAL_DISTANCE)
            else -Units.inchesToMeters(ReefPositioning.RIGHT_REEF_LATERAL_DISTANCE)
        } else 0.0
        return@run Transform2d(
            lateral,
            -Units.inchesToMeters(ReefPositioning.PERPENDICULAR_DISTANCE_FROM_REEF + reefDistance.distance),
            Rotation2d()
        )
    }
    override val pose: Pose2d = run {
        return@run AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[aprilTag]!!.transformBy(transformation)
    }

}

/**
 * Enum class representing the distance from the reef.
 * This is used to determine how far the robot should be from the reef wall when placing coral.
 * @property distance The distance in meters.
 */
enum class ReefDistance(val distance: Double) {
    CLOSE(0.0),
    MEDIUM(21.0),
    FAR(27.0)
}

