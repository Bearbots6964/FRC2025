package frc.robot.automation.drivebase.requests

import edu.wpi.first.math.geometry.Transform2d

/**
 * An interface that extends [LocationRequest] to represent a point on the field that is anchored to an AprilTag.
 * This, too, is mostly backend. Most use cases will be handled by higher-level abstractions.
 *
 * @param aprilTag The ID of the AprilTag that this point is anchored to.
 * @param transformation The transformation from the AprilTag's pose to this point's pose.
 * @property pose The pose of the anchored point, which is derived from the AprilTag's position and the transformation. Likely should not be used.
 */
sealed interface AnchoredPointRequest : LocationRequest {
    val aprilTag: Int
    val transformation: Transform2d
}

