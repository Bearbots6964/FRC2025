package frc.robot.util

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import java.util.concurrent.locks.Lock

class CommandQueue {
    private val queue = mutableListOf<() -> Command>()
    private var currentCommand: Command = Commands.none()

    fun add(command: () -> Command) {
        if (queue.isEmpty()) {
            queue.add(command)
            fireCommand()
        } else {
            queue.add(command)
        }
    }


    fun clearAll() {
        queue.clear()
        if (currentCommand.isScheduled) {
            currentCommand.cancel()
        }
    }

    private fun fireCommand() {
        if (queue.isNotEmpty()) {
            synchronized(currentCommand) {
                currentCommand = queue.removeAt(0).invoke()
                currentCommand.andThen(Commands.runOnce(this::commandFinishedCallback)).schedule()
            }
        }
    }

    private fun commandFinishedCallback() {
        // empty for now
    }
}