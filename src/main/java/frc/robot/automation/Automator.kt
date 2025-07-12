package frc.robot.automation

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

/**
 * Interface for an automator that processes requests.
 */
interface Automator {
    var running: Boolean
    fun accept(request: Request): Command
}