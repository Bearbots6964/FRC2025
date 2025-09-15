package frc.robot.util

import edu.wpi.first.util.protobuf.ProtobufSerializable
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import us.hebi.quickbuf.ProtoMessage
import java.io.Serializable
import java.nio.ByteBuffer

/**
 * A utility class for managing and executing a queue of commands sequentially.
 * This class implements the [Sendable] interface, allowing its state to be monitored
 * and controlled via the SmartDashboard.
 * WPILib-approved.
 * Yes, I checked with the maintainers.
 */
class CommandQueue : Sendable {

    /**
     * The internal queue holding the commands to be executed. Each element in the
     * list is a lambda expression that returns a [Command]. This allows for lazy
     * instantiation of commands.
     */
    val queue = mutableListOf<() -> Command>()

    /**
     * The command that is currently being executed by the queue.
     */
    private var currentCommand: Command = Commands.none()

    /**
     * An optional special message associated with the queue's current state.
     * Defaults to "STOPPED". This message is often used to indicate a specific state,
     * such as when the queue is paused.
     */
    var state: CommandQueueState = CommandQueueState.IDLE

    /**
     * The name of this command queue, used for identification in logging and the
     * SmartDashboard.
     * Defaults to "CommandQueue (name me!)".
     */
    var name: String = "CommandQueue"

    final val struct = CommandQueueStruct()

    private var callbackCommand: () -> Command = Commands::none
    fun withCallback(command: () -> Command): CommandQueue {
        callbackCommand = command
        return this
    }

    /**
     * Adds one or more commands to the queue. If the queue was previously empty,
     * the first command added will be started immediately.
     *
     * @param command A vararg of lambda expressions, each returning a [Command].
     */
    fun add(vararg command: () -> Command) {
        if (queue.isEmpty()) {
            queue.addAll(command)
            state = CommandQueueState.RUNNING
            fireCommand()
        } else {
            queue.addAll(command)
        }
    }

    /**
     * Adds one or more commands to the queue and returns a [Command] that, when
     * scheduled, will start the execution of the queue if it is currently idle.
     * This is useful for adding a sequence of commands that should be triggered
     * by another command or event.
     *
     * @param command A vararg of lambda expressions, each returning a [Command].
     * @return A [Command] that, when run once, will initiate the execution of the added commands if the queue is idle.
     */
    fun addAsCommand(vararg command: () -> Command): Command = Commands.runOnce({
        if ((state == CommandQueueState.IDLE) || (state == CommandQueueState.INTERRUPTED)) {
            queue.addAll(command)
            state = CommandQueueState.RUNNING
            fireCommand()
        } else {
            queue.addAll(command)
        }
    })


    /**
     * Adds one or more commands to the queue but does not start their execution
     * immediately, even if the queue is currently idle. This is useful for
     * pre-populating the queue before a specific trigger starts the execution.
     *
     * @param command A vararg of lambda expressions, each returning a [Command].
     */
    fun addButDoNotStart(vararg command: () -> Command) {
        queue.addAll(command)
    }

    fun addButDoNotStartAsCommand(vararg command: () -> Command): Command {
        return Commands.runOnce({ queue.addAll(command) }).alongWith(callbackCommand.invoke())
    }

    /**
     * Pauses the execution of the command queue. If a command is currently running,
     * it will be canceled. The [specialMessage] will be set to "PAUSED".
     */
    fun pause() {
        state = CommandQueueState.PAUSED
        if (currentCommand.isScheduled) {
            currentCommand.cancel()
        }
    }

    /**
     * Starts the execution of the command queue if it is not currently running
     * and there are commands in the queue. If the queue is already running or
     * empty, this method has no effect.
     */
    fun start() {
        if (queue.isNotEmpty()) {
            state = CommandQueueState.RUNNING
            fireCommand()
        }
    }


    /**
     * Clears all commands from the queue and cancels the currently executing command,
     * if any. This effectively stops any pending or ongoing command execution.
     */
    fun clearAll() {
        queue.clear()
        if (currentCommand.isScheduled) {
            currentCommand.cancel()
        }
        state = CommandQueueState.IDLE
    }

    fun clearAllAsCommand(): Command = Commands.runOnce({
        queue.clear()
        if (currentCommand.isScheduled) {
            currentCommand.cancel()
        }
        state = CommandQueueState.IDLE
    })

    /**
     * Cancels the command that is currently being executed by the queue. If no
     * command is running, this method has no effect.
     * Automatically starts next command.
     */
    fun cancelCurrent() {
        if (currentCommand.isScheduled) {
            currentCommand.cancel()
            state =
                CommandQueueState.RUNNING // to offset INTERRUPTED set by the finallyDo() decorator
        }
    }

    /**
     * Sets a special message for the command queue. This message can be used to
     * communicate the current state or any relevant information about the queue.
     *
     * @param message The string to set as the special message.
     */

    /**
     * Sets the name of the command queue. This name is used for identification,
     * such as in logging and on the SmartDashboard.
     *
     * @param name The name to set for the command queue.
     */

    /**
     * Returns the name of the command queue. This overrides the default [toString]
     * method to provide a more meaningful representation of the object.
     *
     * @return The name of the command queue.
     */
    override fun toString(): String {
        return name
    }

    /**
     * Executes the next command in the queue. This method retrieves the first
     * command from the queue, sets it as the [currentCommand], and schedules it
     * to run. Once the command finishes, the [commandFinishedCallback] will be
     * invoked. If the queue is empty, the [idle] flag is set to true.
     */
    private fun fireCommand() {
        if (queue.isNotEmpty()) {
            state = CommandQueueState.RUNNING
            currentCommand = queue.removeAt(0).invoke()
            currentCommand.finallyDo(this::commandFinishedCallback)
                .handleInterrupt { state = CommandQueueState.INTERRUPTED }.schedule()
        } else state = CommandQueueState.IDLE
    }

    /**
     * Returns true if the command queue is currently empty (contains no pending commands).
     *
     * @return True if the queue is empty, false otherwise.
     */
    fun isEmpty() = queue.isEmpty()

    /**
     * Returns true if the command queue is currently idle (not executing any command
     * and the queue is empty).
     *
     * @return True if the queue is idle, false otherwise.
     */
    fun isIdle() = state == CommandQueueState.IDLE

    /**
     * A callback method that is invoked when the currently executing command finishes.
     * If the queue is not paused, this method will call [fireCommand] to start the
     * next command in the queue.
     */
    private fun commandFinishedCallback() {
        state = CommandQueueState.IDLE
    }

    /**
     * Initializes this [Sendable] object for use with the SmartDashboard. It sets
     * up properties to display the names and IDs of the commands in the queue,
     * as well as a mechanism to cancel commands directly from the dashboard.
     *
     * @param builder The [SendableBuilder] used to configure the properties.
     */
    override fun initSendable(builder: SendableBuilder?) {
        // Stolen from the CommandScheduler class
        builder?.setSmartDashboardType("Scheduler")
        builder?.addStringArrayProperty(
            "Names", {
                val arr = queue.map { it.invoke().name }.toTypedArray()
                arrayOf(state.toString(), *arr)
            }, null
        )
        builder?.addIntegerArrayProperty(
            "Ids", {
                (LongRange(1L, queue.size.toLong() + 1L).toSet()
                    .toLongArray()) // compensate for whether there is a special message or not
            }, null
        )
        builder?.addIntegerArrayProperty(
            "Cancel", { LongArray(0) }) { longs ->
            longs.forEach {
                // ditto here
                if (it.toInt() > 1) queue.removeAt(it.toInt() - 2).invoke().cancel()
                else clearAll()
            }
        }
    }

    /**
     * Sets the name of the command queue. This is a fluent method that returns the
     * current [CommandQueue] instance, allowing for method chaining.
     * @param name The name to set for the command queue.
     * @return The current [CommandQueue] instance.
     */
    fun withName(name: String): CommandQueue {
        this.name = name
        return this
    }

    /**
     * Contains static utility methods and constants related to the [CommandQueue] class.
     */
    companion object {
        /**
         * A constant string used as the default message to indicate that a special
         * command has finished its execution.
         */
        private const val SPECIAL_COMMAND_FINISHED = "CALLBACK FINISHED"

        /**
         * Creates a command that waits until a special command in the [otherQueue]
         * sets its special message to [SPECIAL_COMMAND_FINISHED]. This can be used
         * for synchronization between different command queues.
         *
         * @param otherQueue The command queue to monitor for the special message.
         * @return A [Command] that will remain active until the specified special
         * message is set in the other command queue.
         */
        @JvmStatic
        fun waitUntilSpecialCommandFinishes(otherQueue: CommandQueue): Command =
            // Wait until the other queue is in a callback state...
            Commands.waitUntil { otherQueue.state == CommandQueueState.CALLBACK }
                // and then set the state of this queue to running
                .andThen(Commands.runOnce({ otherQueue.state = CommandQueueState.RUNNING }))
                .withName("Waiting for ${otherQueue.name}...")

        /**
         * Creates a special command that, when executed within a [CommandQueue],
         * will set its [state] to [CommandQueueState.CALLBACK]. This is often
         * used in conjunction with [waitUntilSpecialCommandFinishes] to signal the
         * completion of a specific task or state within a command queue.
         *
         * @param queueToSignal The command queue that this special command belongs
         * to and will signal upon completion by setting its status.
         * @return A lambda expression that creates a [Command] which, when executed,
         * sets the status of the specified queue.
         */
        @JvmStatic
        fun createSpecialCommand(
            queueToSignal: CommandQueue
        ): () -> Command = {
            Commands.runOnce({
                queueToSignal.state = CommandQueueState.CALLBACK
            }).withName("Signal ${queueToSignal.name} with callback")
        }
    }

    /**
     * Represents the different states that a CommandQueue can be in.
     * This enum is used to track and communicate the current operational status
     * of a CommandQueue instance.
     */
    enum class CommandQueueState {
        /**
         * Indicates that the queue is empty and not executing any commands.
         */
        IDLE {
            /**
             * Returns a string representation of the `IDLE` state.
             * @return "Idle..."
             */
            override fun toString(): String {
                return "Idle..."
            }
        },

        /**
         * Indicates that the queue is actively executing commands.
         */
        RUNNING {
            /**
             * Returns a string representation of the `RUNNING` state.
             * @return "Running..."
             */
            override fun toString(): String {
                return "Running..."
            }
        },

        /**
         * Indicates that the queue's execution has been temporarily suspended.
         */
        PAUSED {
            /**
             * Returns a string representation of the `PAUSED` state.
             * @return "Paused!"
             */
            override fun toString(): String {
                return "Paused!"
            }
        },

        /**
         * Indicates that the queue is in a callback state, typically used for synchronization
         * between different command queues.
         */
        CALLBACK {
            /**
             * Returns a string representation of the `CALLBACK` state.
             * @return "Callback!"
             */
            override fun toString(): String {
                return "Callback!"
            }
        },

        /**
         * Indicates that the queue's execution was interrupted, typically due to
         * a command being cancelled.
         */
        INTERRUPTED {
            /**
             * Returns a string representation of the `INTERRUPTED` state.
             * @return "Interrupted!"
             */
            override fun toString(): String {
                return "Interrupted!"
            }
        }
    }
    fun dumpFromStruct(names: Array<String>) {
        queue.addAll(names.map { name -> { Commands.runOnce({ println("$name run from struct") }) }})
    }
}