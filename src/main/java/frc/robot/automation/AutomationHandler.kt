package frc.robot.automation

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.automation.drivebase.DrivebaseAutomator
import frc.robot.automation.drivebase.requests.DriveRequest
import frc.robot.automation.superstructure.SuperstructureAutomator
import frc.robot.automation.superstructure.requests.SuperstructureRequest
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.arm.ClawIntake
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.elevator.Elevator

class AutomationHandler(
    val subsystems: SubsystemData,
) {
    val superstructureAutomator: SuperstructureAutomator = SuperstructureAutomator(subsystems)
    val drivebaseAutomator: DrivebaseAutomator = DrivebaseAutomator(subsystems)
    fun accept(vararg requests: Request): Command {
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