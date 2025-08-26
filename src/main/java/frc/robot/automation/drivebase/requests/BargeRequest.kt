package frc.robot.automation.drivebase.requests

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.automation.drivebase.BargePositioning
import frc.robot.automation.states.BargePosition

/**
 * A request to position the robot at a specific barge location.
 */
class BargeRequest(val bargePosition: BargePosition) : LocationRequest {
    override val pose: Pose2d = run {
        var position: Pose2d = when (bargePosition) {
            BargePosition.LEFT -> BargePositioning.leftBargePosition
            BargePosition.MIDDLE -> BargePositioning.middleBargePosition
            BargePosition.RIGHT -> BargePositioning.rightBargePosition
            BargePosition.NONE -> Pose2d()
        }
        if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get().equals(
                DriverStation.Alliance.Red
            )
        ) {
            position = FlippingUtil.flipFieldPose(position)
        }
        position
    }
}