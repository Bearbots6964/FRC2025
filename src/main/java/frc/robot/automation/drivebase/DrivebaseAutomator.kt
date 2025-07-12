package frc.robot.automation.drivebase

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import frc.robot.automation.Automator
import frc.robot.automation.Request
import frc.robot.automation.SubsystemData
import frc.robot.automation.drivebase.requests.BrakeRequest
import frc.robot.automation.drivebase.requests.CoralStationRequest
import frc.robot.automation.drivebase.requests.DriveRequest
import frc.robot.automation.drivebase.requests.ReefRequest
import frc.robot.subsystems.drive.Drive
import java.util.function.Supplier

/**
 * Automator for the drivebase subsystem.
 * This is where the bulk of the drivebase logic is handled.
 * It processes requests related to driving, such as pathfinding and braking.
 * It uses the [Drive] subsystem to execute commands based on the requests it receives.
 * This class is responsible for interpreting the requests and executing the appropriate commands on the drivebase.
 */
class DrivebaseAutomator : Automator {
    val subsystem: Drive
    val joystick: Supplier<Translation2d>

    /**
     * Constructs a DrivebaseAutomator with the provided subsystem data.
     *
     * @param data The subsystem data containing the drivebase and joystick supplier.
     */
    constructor(
        data: SubsystemData
    ) {
        subsystem = data.drivebase
        joystick = data.nudge
    }

    /**
     * Extracts and interprets a request for the drivebase to do something.
     *
     * @param request The request to be processed, which can be a [ReefRequest], [BrakeRequest], or [CoralStationRequest].
     * @return A command that represents the action to be taken by the drivebase.
     */
    fun extractRequest(
        request: DriveRequest
    ): Command {
        return when (request) {
            is ReefRequest -> subsystem.followRepulsorField(request.pose, joystick).asProxy()
            is BrakeRequest -> ScheduleCommand(
                Commands.runOnce(
                    { subsystem.stopWithX() },
                    subsystem
                ).withName("Brake")
            )

            is CoralStationRequest -> subsystem.followRepulsorField(request.pose, joystick)
                .asProxy()
        }
    }

    override var running: Boolean = false

    /**
     * Accepts a request and processes it to return a command.
     * If the request is not a [DriveRequest], it returns a no-op command.
     *
     * @param request The request to be processed.
     * @return A command that represents the action to be taken by the drivebase.
     */
    override fun accept(request: Request): Command {
        if (request !is DriveRequest) return Commands.none()
        return Commands.sequence(
            Commands.runOnce({ running = true }),
            extractRequest(request)
        ).finallyDo(Runnable { running = false })
    }

    /**
     * Creates a command to drive the robot to a specific position to score Coral on the Reef.
     *
     * @param placement The placement of the coral on the Reef, correlating to the position of each post.
     */
    fun coralPosition(placement: CoralPlacement): Command {
        return accept(ReefRequest(placement))
    }

    /**
     * Creates a command to drive the robot to a specific position to place algae.
     *
     * @param placement The placement of the algae, correlating to the position of each algae station.
     */
    fun algaePosition(placement: AlgaePlacement): Command {
        return accept(ReefRequest(placement))
    }

    /**
     * Creates a command to drive the robot to a Coral Station, optionally nudging it in a specified direction.
     * @param side The side of the Coral Station to approach.
     * @param nudge The direction to nudge the robot parallel to the Coral Station, defaulting to NONE.
     */
    fun coralStation(side: CoralStation, nudge: CoralStationNudgeDirection = CoralStationNudgeDirection.NONE): Command {
        return accept(CoralStationRequest(side, nudge))
    }

    fun setPathfindingSpeed(speed: Double): Command {
        return Commands.runOnce({ subsystem.setPathfindingSpeedPercent(speed) })
    }

    fun backUpQuickly(): Command {
        return subsystem.backUpFaster().asProxy()
    }
}
