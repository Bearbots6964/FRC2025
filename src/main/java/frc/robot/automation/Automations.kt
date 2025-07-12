package frc.robot.automation

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import frc.robot.automation.drivebase.AlgaePlacement
import frc.robot.automation.drivebase.CoralPlacement
import frc.robot.automation.drivebase.CoralStation
import frc.robot.automation.drivebase.CoralStationNudgeDirection
import frc.robot.automation.drivebase.DrivebaseAutomator
import frc.robot.automation.drivebase.PathfindingConstants
import frc.robot.automation.drivebase.requests.BrakeRequest
import frc.robot.automation.drivebase.requests.ReefDistance
import frc.robot.automation.drivebase.requests.ReefRequest
import frc.robot.automation.states.AlgaeStatus
import frc.robot.automation.states.AutoTask
import frc.robot.automation.states.CoralStatus
import frc.robot.automation.superstructure.Positions
import frc.robot.automation.superstructure.SuperstructureAutomator
import frc.robot.automation.superstructure.requests.SuperstructureRequest

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
    fun coralCycle(): Command {
        var superstructureInPosition = false
        return sequence(
            setStatus(task = AutoTask.TO_CORAL_STATION),
            defer({
                drivebase.coralStation(
                    state.nextStation, CoralStationNudgeDirection.OUTWARD
                )
            }, null).deadlineFor(superstructure.goToPreCoralPickup()),

            setStatus(task = AutoTask.WAITING),
            drivebase.accept(BrakeRequest())
                .withDeadline(waitUntil({ state.coralStatus == CoralStatus.ON_INTAKE })),

            setStatus(task = AutoTask.TO_REEF),
            drivebase.setPathfindingSpeed(PathfindingConstants.CORAL_INTAKE_SPEED),

            parallel(
                // Turn up speed once coral is secure
                waitUntil { state.coralStatus == CoralStatus.IN_CLAW }.andThen(
                    drivebase.setPathfindingSpeed(
                        PathfindingConstants.TO_REEF_SPEED
                    )
                ),

                waitUntil { superstructure.grabbed }.andThen(setStatus(coralStatus = CoralStatus.IN_CLAW)),

                sequence(
                    defer({
                        drivebase.accept(
                            ReefRequest(
                                state.nextReef, ReefDistance.MEDIUM
                            )
                        )
                    }, null),
                    waitUntil { superstructureInPosition }.deadlineFor(drivebase.accept(BrakeRequest()))
                        .andThen(defer({
                            drivebase.accept(
                                ReefRequest(
                                    state.nextReef, ReefDistance.CLOSE
                                )
                            )
                        }, null))
                ),

                sequence(
                    superstructure.pickUpCoral()
                        .onlyIf { state.coralStatus == CoralStatus.ON_INTAKE },
                    waitUntil(drivebase.subsystem::nearGoal),
                    defer({
                        superstructure.goToL4()
                    }, null),
                    runOnce({
                        superstructureInPosition = true
                    })
                )
            ),

            defer({
                superstructure.scoreAt(state.nextState).alongWith(drivebase.backUpQuickly())
                    .andThen(setStatus(coralStatus = CoralStatus.NONE))
                    .onlyIf { state.coralStatus == CoralStatus.IN_CLAW }
            }, null)
        ).finallyDo(Runnable { setStatus(task = AutoTask.IDLE) })
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
