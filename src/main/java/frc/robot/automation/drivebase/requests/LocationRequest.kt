package frc.robot.automation.drivebase.requests

import edu.wpi.first.math.geometry.Pose2d


/**
 * Base interface which all classes pertaining to a location on the field should implement.
 * This is used to represent a location in the field for the drivebase automator to ingest.
 * This interface consists of a single property, `pose`.
 * As a [Pose2d], it's as basic as it gets in regard to representing a location.
 */
sealed interface LocationRequest : DriveRequest {
    /**
     * The pose of the location on the field.
     * This is a [edu.wpi.first.math.geometry.Pose2d] object that represents the position and orientation of the location.
     */
    val pose: Pose2d
}
