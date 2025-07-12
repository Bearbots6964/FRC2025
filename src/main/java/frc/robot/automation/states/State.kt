package frc.robot.automation.states

import frc.robot.automation.drivebase.CoralStation
import frc.robot.automation.drivebase.UnifiedReefLocation
import frc.robot.automation.superstructure.Positions
import frc.robot.automation.superstructure.SuperstructureConstants
import frc.robot.commands.PathfindingFactories
import frc.robot.util.annotation.Logged
import org.littletonrobotics.junction.Logger

/**
 * Data class representing the state of the robot during autonomous operations.
 * It holds information about coral and algae status, target Position, and the current task.
 *
 * @property coralStatus The current status of the coral (NONE, ON_INTAKE, IN_CLAW).
 * @property algaeStatus The current status of the algae (NONE, IN_CLAW).
 * @property nextState The target superstructure state.
 * @property nextReef The target reef for pathfinding.
 * @property nextBarge The target barge position.
 * @property nextStation The target coral station side.
 * @property nextCage The target cage position.
 * @property nextAlgaePosition The target reef for algae collection.
 * @property task The current autonomous task being executed.
 */
class State(
    var coralStatus: CoralStatus,
    var algaeStatus: AlgaeStatus,
    var nextState: Positions,
    var currentState: Positions,
    var nextReef: UnifiedReefLocation,
    var nextStation: CoralStation,
    var nextAlgaePosition: UnifiedReefLocation,
    var task: AutoTask
) : ShortString {

    //var io: StateIOAutoLogged

    /**
     * Constructor for the State class with default values.
     * Initializes all properties to NONE or IDLE.
     */
    constructor() : this(
        CoralStatus.NONE,
        AlgaeStatus.NONE,
        Positions.HOME,
        Positions.HOME,
        UnifiedReefLocation.NONE,
        CoralStation.LEFT,
        UnifiedReefLocation.NONE,
        AutoTask.IDLE
    )

    /**
     * Creates a new State object with specified values, defaulting to the current state's values if not provided.
     *
     * @param coralStatus The coral status to set. Defaults to the current coralStatus.
     * @param algaeStatus The algae status to set. Defaults to the current algaeStatus.
     * @param nextState The next superstructure state to set. Defaults to the current nextState.
     * @param nextReef The next reef to set. Defaults to the current nextReef.
     * @param nextBarge The next barge position to set. Defaults to the current nextBarge.
     * @param nextStation The next coral station side to set. Defaults to the current nextStation.
     * @param nextCage The next cage position to set. Defaults to the current nextCage.
     * @param nextAlgaePosition The next algae position to set. Defaults to the current nextAlgaePosition.
     * @param task The autonomous task to set. Defaults to the current task.
     * @return A new State object with the specified or default values.
     */
    fun push(
        coralStatus: CoralStatus = this.coralStatus,
        algaeStatus: AlgaeStatus = this.algaeStatus,
        nextState: Positions = this.nextState,
        currentState: Positions = this.currentState,
        nextReef: UnifiedReefLocation = this.nextReef,
        nextStation: CoralStation = this.nextStation,
        nextAlgaePosition: UnifiedReefLocation = this.nextAlgaePosition,
        task: AutoTask = this.task
    ): State {
        this.coralStatus = coralStatus
        this.algaeStatus = algaeStatus
        this.nextState = nextState
        this.currentState = currentState
        this.nextReef = nextReef
        this.nextStation = nextStation
        this.nextAlgaePosition = nextAlgaePosition
        this.task = task
        return this
    }

    /**
     * Updates and logs this state. Intended to be run every loop.
     */
    fun updateIO() {
        val io = LoggedStateIO()
        io.coralStatus = coralStatus
        io.algaeStatus = algaeStatus
        io.nextState = nextState
        io.currentState = currentState
        io.nextReef = nextReef
        io.nextStation = nextStation
        io.nextAlgaePosition = nextAlgaePosition
        io.task = task
        io.stateString = toString()
        io.hasAlgae = algaeStatus != AlgaeStatus.NONE
        io.hasCoral = coralStatus != CoralStatus.NONE
        io.shortStateString = toShortString()
        io.shortTaskString = task.toShortString()
        io.shortStatusString = coralStatus.toShortString() + algaeStatus.toShortString()
        io.shortNextStateString = nextState.toShortString()
        io.shortReefString = nextReef.toShortString() + nextAlgaePosition.toShortString()
        io.shortBargeString = "" // No barge/cage in new signature
        io.shortCoralStationString = nextStation.toShortString()
        io.coralInClaw = coralStatus == CoralStatus.IN_CLAW

        Logger.processInputs("State", io)
    }

    companion object {
        /**
         * Creates a new State object with specified values.
         *
         * @param coralStatus The coral status to set. Defaults to NONE.
         * @param algaeStatus The algae status to set. Defaults to NONE.
         * @param nextState The next superstructure state to set. Defaults to HOME.
         * @param nextReef The next reef to set. Defaults to NONE.
         * @param nextBarge The next barge position to set. Defaults to NONE.
         * @param nextStation The next coral station side to set. Defaults to NONE.
         * @param nextCage The next cage position to set. Defaults to NONE.
         * @param nextAlgaePosition The next algae position to set. Defaults to NONE.
         * @param task The autonomous task to set. Defaults to IDLE.
         * @return A new State object with the specified values.
         */
        fun new(
            coralStatus: CoralStatus = CoralStatus.NONE,
            algaeStatus: AlgaeStatus = AlgaeStatus.NONE,
            nextState: Positions = Positions.HOME,
            currentState: Positions = Positions.HOME,
            nextReef: UnifiedReefLocation = UnifiedReefLocation.NONE,
            nextStation: CoralStation = CoralStation.LEFT,
            nextAlgaePosition: UnifiedReefLocation = UnifiedReefLocation.NONE,
            task: AutoTask = AutoTask.IDLE
        ): State {
            return State(
                coralStatus,
                algaeStatus,
                nextState,
                currentState,
                nextReef,
                nextStation,
                nextAlgaePosition,
                task
            )
        }
    }

    /**
     * Returns a string representation of the current task.
     *
     * @return A string describing the current task and related parameters.
     */
    override fun toString(): String {
        when (task) {
            AutoTask.IDLE -> return "Idle ($coralStatus, $algaeStatus)"
            AutoTask.TO_CORAL_STATION -> {
                return when (nextStation) {
                    CoralStation.LEFT -> "Pathfind to Left Coral Station (Arm @ $currentState, $coralStatus)"
                    CoralStation.RIGHT -> "Pathfind to Right Coral Station (Arm @ $currentState, $coralStatus)"
                }
            }

            AutoTask.TO_REEF -> {
                return "Pathfind to Reef $nextReef (Arm @ $currentState, $coralStatus, $algaeStatus)"
            }

            AutoTask.TO_BARGE -> {
                return "Pathfind to Barge (Arm @ $currentState, $coralStatus, $algaeStatus)"
            }

            AutoTask.TO_CAGE -> {
                return "Pathfind to Cage ($coralStatus, $algaeStatus)"
            }

            AutoTask.TO_ALGAE -> {
                return "Pathfind to Algae $nextAlgaePosition ($coralStatus, $algaeStatus)"
            }

            AutoTask.WAITING -> return "Waiting for input... ($coralStatus, $algaeStatus)"

            AutoTask.SPIT_OUT_ALGAE -> {
                return "Spit Out Algae ($coralStatus, $algaeStatus)"
            }

            else -> {
                return "Unknown task, something has gone very wrong!"
            }
        }
    }

    override fun toShortString(): String {
        return task.toShortString() + "|" +
                coralStatus.toShortString() + algaeStatus.toShortString() + "|" +
                currentState.toShortString() + "|" +
                nextReef.toShortString()  + nextStation.toShortString() +  nextAlgaePosition.toShortString()
    }
}
// this stupid annotation setup took an hour of my life that I'm never getting back
@Logged
open class StateIO {
    var coralStatus: CoralStatus = CoralStatus.NONE
    var algaeStatus: AlgaeStatus = AlgaeStatus.NONE
    var nextState: Positions = Positions.HOME
    var currentState: Positions = Positions.HOME
    var nextReef: UnifiedReefLocation = UnifiedReefLocation.NONE
    var nextStation: CoralStation = CoralStation.LEFT
    var nextAlgaePosition: UnifiedReefLocation = UnifiedReefLocation.NONE
    var task: AutoTask = AutoTask.IDLE
    var stateString: String = ""
    var shortStateString: String = ""
    var shortTaskString: String = ""
    var shortStatusString: String = ""
    var shortNextStateString: String = ""
    var shortReefString: String = ""
    var shortBargeString: String = ""
    var shortCoralStationString: String = ""
    var hasAlgae: Boolean = false
    var hasCoral: Boolean = false
    var coralInClaw: Boolean = false
}
