package frc.robot.util

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

// I ran this by one of the WPILib developers and he said it was probably fine
class CommandQueue : Sendable {
    private val queue = mutableListOf<() -> Command>()
    private var currentCommand: Command = Commands.none()

    fun add(vararg command: () -> Command) {
        if (queue.isEmpty()) {
            queue.addAll(command)
            fireCommand()
        } else {
            queue.addAll(command)
        }
    }


    fun clearAll() {
        queue.clear()
        if (currentCommand.isScheduled) {
            currentCommand.cancel()
        }
    }

    fun cancelCurrent() {
        if (currentCommand.isScheduled) {
            currentCommand.cancel()
        }
    }

    private fun fireCommand() {
        if (queue.isNotEmpty()) {
            currentCommand = queue.removeAt(0).invoke()
            currentCommand.finallyDo(this::commandFinishedCallback).schedule()
        }
    }

    fun isEmpty() = queue.isEmpty()

    private fun commandFinishedCallback() {
        fireCommand()
    }

    /**
     * Initializes this [Sendable] object.
     *
     * @param builder sendable builder
     */
    override fun initSendable(builder: SendableBuilder?) {
        // Stolen from the CommandScheduler class
        builder?.setSmartDashboardType("Scheduler")
        builder?.addStringArrayProperty(
            "Names", { queue.map { it.invoke().name }.toTypedArray() }, null
        )
        builder?.addIntegerArrayProperty(
            "Ids", { (LongRange(1L, queue.size.toLong()).toSet().toLongArray()) }, null
        )
        builder?.addIntegerArrayProperty(
            "Cancel", { LongArray(0) }) { longs ->
            longs.forEach {
                queue.removeAt(it.toInt()).invoke().cancel()
            }
        }
    }
}