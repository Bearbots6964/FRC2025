package frc.robot.automation

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.Constants
import frc.robot.automation.drivebase.CoralStationPositioning
import frc.robot.automation.drivebase.DrivebaseAutomator
import frc.robot.automation.drivebase.PathfindingConstants
import frc.robot.automation.drivebase.requests.BargeRequest
import frc.robot.automation.drivebase.requests.BrakeRequest
import frc.robot.automation.drivebase.requests.CoralStationRequest
import frc.robot.automation.drivebase.requests.DriveBackUpRequest
import frc.robot.automation.drivebase.requests.ReefDistance
import frc.robot.automation.drivebase.requests.ReefRequest
import frc.robot.automation.drivebase.requests.Speed
import frc.robot.automation.states.AlgaeStatus
import frc.robot.automation.states.AutoTask
import frc.robot.automation.states.BargePosition
import frc.robot.automation.states.CoralStatus
import frc.robot.automation.superstructure.Position
import frc.robot.automation.superstructure.SuperstructureAutomator
import frc.robot.automation.superstructure.requests.IntakeState
import frc.robot.automation.superstructure.requests.SuperstructurePickUpCoralRequest
import frc.robot.automation.superstructure.requests.SuperstructureRequest
import frc.robot.automation.superstructure.requests.SuperstructureStateRequest
import frc.robot.automation.superstructure.requests.SuperstructureScoreRequest

/**
 * The Automations class orchestrates the robot's autonomous operations by coordinating the drivebase and superstructure subsystems.
 * It provides methods to automate tasks such as picking up coral game pieces and delivering them to scoring locations.
 *
 * This is the highest level class for managing autonomous operations in the robot.
 *
 * @property data The subsystem data containing references to the robot's subsystems and state.
 */
class Automations(val data: SubsystemData) {
    val superstructure = SuperstructureAutomator(data)
    val drivebase = DrivebaseAutomator(data)
    val state
        get() = data.state

    /**
     * Automates the process of picking up a coral game piece and delivering it to a scoring location (the "coral cycle").
     *
     * This method returns a WPILib Command that sequences and coordinates the robot's drivebase and superstructure
     * to perform the following high-level steps:
     *
     * 1. **Navigate to the Coral Station:**
     *    The robot drives to the specified coral station (using the current state's `nextStation`), while simultaneously
     *    preparing the superstructure for coral pickup.
     *
     * 2. **Wait for Coral Detection:**
     *    Once at the station, the robot waits until the coral is detected on the intake, then applies the brakes to hold position.
     *
     * 3. **Prepare for Scoring:**
     *    The robot sets its pathfinding speed for carrying coral and prepares to move toward the reef (scoring area).
     *
     * 4. **Coordinate Pickup and Movement:**
     *    Several actions happen in parallel:
     *      - The robot increases speed once the coral is secured in the claw.
     *      - The superstructure updates the coral status when the coral is grabbed.
     *      - The drivebase navigates to the reef, first at a medium distance, then closes in for precise placement.
     *      - The superstructure picks up the coral, waits until the robot is near the goal, then moves to the scoring position.
     *
     * 5. **Score the Coral:**
     *    The superstructure scores the coral at the desired level, while the drivebase quickly backs up. The coral status is reset.
     *
     * 6. **Cleanup:**
     *    The automation finishes by setting the robot's task status to idle.
     *
     * Throughout the process, the method uses WPILib's command-based framework to sequence, parallelize, and conditionally
     * execute actions based on the robot's sensors and internal state.
     *
     * @return A Command that performs the full coral cycle automation.
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html">WPILib Command Compositions</a>
     */
    fun coralCycle(waitTime: Boolean = false): Command {
        var superstructureInPosition = false
        return sequence(
            pickUpCoral(waitTime),
            scoreCoral(),
        ).finallyDo(Runnable { setStatus(task = AutoTask.IDLE) })
    }

    fun algaeCycle(): Command {
        return sequence(
            // set state
            setStatus(task = AutoTask.TO_ALGAE),
            // drive to the algae the drivers select - only read this at runtime
            defer(
                {
                    drivebase.accept(
                        ReefRequest(
                            state.nextAlgaePosition, ReefDistance.FAR
                        )
                    )
                },
                setOf<Subsystem>()
            ),
            // move superstructure to the algae intake position; brake the drivebase
            defer({
                superstructure.accept(SuperstructureStateRequest(state.nextAlgaePosition.toAlgaePosition()))
            }, setOf<Subsystem>()).deadlineFor(
                drivebase.accept(BrakeRequest())
            ),
            // set pathfinding speed for algae intake
            drivebase.setPathfindingSpeed(Constants.PathfindingConstants.algaeGrabSpeed),

            parallel(
                // drive close to the algae and then brake until the algae is in the claw
                defer({
                    drivebase.accept(
                        ReefRequest(
                            state.nextAlgaePosition,
                            ReefDistance.ALGAE
                        )
                    )
                }, setOf<Subsystem>())
                    .andThen(drivebase.accept(BrakeRequest()))
                    .until(state.algaeInClaw),

                // run intake until the algae is in the claw
                superstructure.accept(SuperstructureRequest(intakeSpeed = IntakeState.ALGAE_INTAKE))
                    .until(state.algaeInClaw),

                // wait until the algae is in the claw, then broadcast to everything else
                waitUntil(superstructure.grabbed).andThen(waitSeconds(0.25))
                    .andThen(setStatus(algaeStatus = AlgaeStatus.IN_CLAW))
            ),

            // once the algae is in the claw, we back up
            drivebase.accept(DriveBackUpRequest(Speed.NORMAL)),

            // decide what to do with the algae
            select(
                mapOf(
                    BargePosition.RIGHT to putAlgaeInBarge(),
                    BargePosition.LEFT to putAlgaeInBarge(),
                    BargePosition.MIDDLE to putAlgaeInBarge(),
                    BargePosition.NONE to none()
                )
            ) { state.nextBarge }
        ).finallyDo(Runnable { setStatus(task = AutoTask.IDLE) })

    }


    fun putAlgaeInBarge(): Command {
        return sequence(
            setStatus(task = AutoTask.TO_BARGE),
            drivebase.setPathfindingSpeed(PathfindingConstants.TO_BARGE_SPEED),
            parallel(
                defer({
                    drivebase.accept(
                        BargeRequest(state.nextBarge)
                    )
                }, setOf<Subsystem>()),

                waitSeconds(0.5).andThen(waitUntil(drivebase.nearBarge)).andThen(
                    superstructure.accept(
                        SuperstructureStateRequest(Position.BARGE_LAUNCH)
                    )
                )
            ).deadlineFor(superstructure.accept(SuperstructureRequest(intakeSpeed = IntakeState.ALGAE_INTAKE))),

            drivebase.accept(DriveBackUpRequest(Speed.FORWARD))
                .deadlineFor(superstructure.accept(SuperstructureRequest(intakeSpeed = IntakeState.ALGAE_OUTTAKE))),
            setStatus(algaeStatus = AlgaeStatus.NONE),
            drivebase.accept(DriveBackUpRequest(speed = Speed.NORMAL)),
            superstructure.accept(SuperstructureStateRequest(Position.PRE_CORAL_PICKUP))
                .deadlineFor(drivebase.accept(BrakeRequest()))
                .withTimeout(3.0)
        ).finallyDo(Runnable {
            setStatus(task = AutoTask.IDLE)
            drivebase.setPathfindingSpeed(PathfindingConstants.TO_REEF_SPEED)
        })
            .withName("Put Algae In Barge")
    }

    fun pickUpCoral(waitTime: Boolean): Command {
        return sequence(
            // go to coral station
            setStatus(task = AutoTask.TO_CORAL_STATION),
            defer(
                {
                    drivebase.accept(
                        CoralStationRequest(
                            state.nextStation
                        )
                    )
                },
                setOf<Subsystem>()
                // move superstructure to pre-coral pickup position
            ).deadlineFor(superstructure.accept(SuperstructureStateRequest(Position.PRE_CORAL_PICKUP))),

            // wait for robot to be told the coral is on the intake by the drivers
            setStatus(task = AutoTask.WAITING),
            drivebase.accept(BrakeRequest())
                .withDeadline(
                    if (waitTime) waitSeconds(CoralStationPositioning.timeToWait) else waitUntil(
                        state.coralOnIntake
                    )
                ),

            )
    }

    fun scoreCoral(): Command {
        var superstructureInPosition = false
        return sequence(
            // set pathfinding speed for carrying coral; update state
            setStatus(task = AutoTask.TO_REEF),
            drivebase.setPathfindingSpeed(PathfindingConstants.CORAL_INTAKE_SPEED),

            // drive to reef, get coral in claw, line up, move everything into position for scoring
            parallel(

                // telemetry and state management
                // once we have coral on the intake, we can set the state to reflect that
                waitUntil(superstructure.grabbed).andThen(setStatus(coralStatus = CoralStatus.IN_CLAW)),
                // Turn up speed once coral is secure
                waitUntil(state.coralInClaw).andThen(
                    drivebase.setPathfindingSpeed(
                        PathfindingConstants.TO_REEF_SPEED
                    )
                ),

                // drivebase
                // move to the reef, hang back until the superstructure is ready, and then close in
                sequence(
                    defer({
                        drivebase.accept(
                            ReefRequest(
                                state.nextReef, ReefDistance.MEDIUM
                            )
                        )
                    }, setOf<Subsystem>()),
                    // we call a brake request here just to make sure the robot stops no matter what
                    waitUntil { superstructureInPosition }.deadlineFor(drivebase.accept(BrakeRequest()))
                        .andThen(defer({
                            drivebase.accept(
                                ReefRequest(
                                    state.nextReef, ReefDistance.CLOSE
                                )
                            )
                        }, setOf<Subsystem>()))
                ),

                // superstructure
                // pick up the coral, wait until the robot is close to the reef, and then move into position
                sequence(
                    superstructure.accept(SuperstructurePickUpCoralRequest())
                        .onlyIf(state.coralOnIntake),
                    waitUntil(drivebase.nearPathfindingGoal),
                    defer({
                        superstructure.accept(SuperstructureStateRequest(state.nextPosition))
                    }, setOf<Subsystem>()),
                    runOnce({
                        superstructureInPosition = true
                    })
                )
            ),

            defer({
                superstructure.accept(SuperstructureScoreRequest(state.nextPosition)).alongWith(
                    drivebase.accept(
                        DriveBackUpRequest()
                    )
                )
                    .andThen(setStatus(coralStatus = CoralStatus.NONE))
                    .onlyIf(state.coralInClaw)
            }, setOf<Subsystem>())
        )
    }

    /**
     * Sets the status of the robot's coral and algae states, as well as the current autonomous task.
     * This method is used to update the robot's state during autonomous operations.
     * @param coralStatus The new coral status to set, or null to leave unchanged.
     * @param algaeStatus The new algae status to set, or null to leave unchanged.
     * @param task The new autonomous task to set, or null to leave unchanged.
     */
    fun setStatus(
        coralStatus: CoralStatus? = null, algaeStatus: AlgaeStatus? = null, task: AutoTask? = null
    ): Command {

        return runOnce({
            if (coralStatus != null) {
                state.push(coralStatus = coralStatus)
            }
            if (algaeStatus != null) {
                state.push(algaeStatus = algaeStatus)
            }
            if (task != null) {
                state.push(task = task)
            }
        })
    }
}
