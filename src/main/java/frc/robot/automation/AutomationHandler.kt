package frc.robot.automation

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.automation.drivebase.DrivebaseAutomator
import frc.robot.automation.drivebase.requests.DriveRequest
import frc.robot.automation.superstructure.SuperstructureAutomator
import frc.robot.automation.superstructure.requests.SuperstructureRequest
import frc.robot.automation.superstructure.requests.SuperstructureStateRequest

class AutomationHandler(
    val subsystems: SubsystemData,
) {
    val superstructureAutomator: SuperstructureAutomator = SuperstructureAutomator(subsystems)
    val drivebaseAutomator: DrivebaseAutomator = DrivebaseAutomator(subsystems)
    val automations: Automations = Automations(subsystems)
    fun accept(vararg requests: Request): Command {
        // ensure there is a maximum of one request per type
        val uniqueRequests = requests.distinctBy { it::class.java }
        if (uniqueRequests.size != requests.size) {
            throw IllegalArgumentException("Duplicate request types found: ${requests.map { it::class.java }}\nhint: only one request of each type is allowed")
        }
        return Commands.parallel(
            *requests.map { request ->
                when (request) {
                    is SuperstructureRequest -> superstructureAutomator.accept(request)
                    is DriveRequest -> drivebaseAutomator.accept(request)
                    else -> throw IllegalArgumentException("Unknown request type: ${request::class.java}")
                }
            }.toTypedArray()
        )
    }
}