package frc.robot.automation.superstructure

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.SuperstructureStates
import frc.robot.automation.Automator
import frc.robot.automation.Request
import frc.robot.automation.SubsystemData
import frc.robot.automation.superstructure.requests.IntakeState
import frc.robot.automation.superstructure.requests.SuperstructureFixPivotRequest
import frc.robot.automation.superstructure.requests.SuperstructurePickUpCoralRequest
import frc.robot.automation.superstructure.requests.SuperstructureRequest
import frc.robot.automation.superstructure.requests.SuperstructureStateRequest
import frc.robot.automation.superstructure.requests.SuperstructureScoreRequest
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.arm.ClawIntake
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.elevator.Elevator

class SuperstructureAutomator(
    val data: SubsystemData
) : Automator {
    val arm: Arm = data.arm
    val climber: Climber = data.climber
    val elevator: Elevator = data.elevator
    val intake: ClawIntake = data.intake
    override var running: Boolean = false
    val grabbed: Trigger = intake.grabbedTrigger


    override fun accept(request: Request): Command {
        // if the request is not a superstructure request, we don't handle it
        if (request !is SuperstructureRequest) return Commands.none()
        // if the request is a superstructure request and deals with the climber-arm conflict, we handle it, but in a ~special way~
        if (request is SuperstructureFixPivotRequest) {
            // if we are scoring on L4, we can get rid of the middleman
            // and put the bot in L4 position while stopping the climber from pivoting
            return if (request.scoreOnL4) Commands.parallel(
                Commands.runOnce({ running = true }),
                fixPivotAndScoreL4()
            ).finallyDo(Runnable { running = false })
            // otherwise, just get the superstructure to a safe place
            else Commands.parallel(Commands.runOnce({
                running = true
            }), fixPivot()).finallyDo(Runnable { running = false })
        }
        // if the request is a superstructure score request, we handle it. nothing special here
        if (request is SuperstructureScoreRequest) return Commands.parallel(Commands.runOnce({
            running = true
        }), scoreAt(request.position)).finallyDo(Runnable { running = false })

        // if the request is a superstructure pick up coral request... you know the drill
        if (request is SuperstructurePickUpCoralRequest) {
            return Commands.parallel(
                Commands.runOnce({ running = true }),
                pickUpCoral()
            ).finallyDo(Runnable { running = false })
        }

        // convert state requests to superstructure requests
        if (request is SuperstructureStateRequest) {
            return accept(SuperstructureRequest(request.position.toState()))
        }

        val list: MutableList<Command> = mutableListOf()
        if (request.climberPosition != null) {
            list.add(moveClimber(request.climberPosition))
        }
        if (request.armPosition != null) {
            list.add(moveArm(request.armPosition))
        }
        if (request.elevatorPosition != null) {
            list.add(moveElevator(request.elevatorPosition))
        }
        if (request.intakeSpeed != null) {
            list.add(spinIntake(request.intakeSpeed))
        }
        return Commands.parallel(
            Commands.runOnce({ running = true }), *list.toTypedArray()
        ).finallyDo(Runnable { running = false })
    }


    fun moveArm(to: Double): Command = arm.moveArmToAngle(to).asProxy()

    fun moveClimber(to: Double): Command = climber.pivotToPosition(to).asProxy()

    fun moveElevator(to: Double): Command = elevator.goToPosition(to).asProxy()

    fun spinIntake(state: IntakeState): Command {
        return when (state) {
            IntakeState.INTAKE -> intake.intake().asProxy()
            IntakeState.ALGAE_INTAKE -> intake.intakeWithoutStoppingForAlgae().asProxy()
            IntakeState.OUTTAKE -> intake.outtake().asProxy()
            IntakeState.ALGAE_OUTTAKE -> intake.outtakeMaxSpeed().asProxy()
            IntakeState.STOP -> intake.stop().asProxy()
        }
    }


    fun pickUpCoral(): Command {
        return accept(
            SuperstructureRequest(
                // pre-positioning
                armPosition = SuperstructureStates.CORAL_PICKUP.armPosition,
                elevatorPosition = SuperstructureStates.CORAL_PICKUP.elevatorPosition,
                climberPosition = SuperstructureStates.CLIMBER_POSITION_EXTENDED,
            )
        ).andThen(
            accept(
                SuperstructureRequest( // spin intake
                    intakeSpeed = IntakeState.INTAKE
                )
            ).deadlineFor(
                accept(
                    SuperstructureRequest( // move climber inwards until secured
                        climberPosition = SuperstructureStates.CLIMBER_POSITION_INTAKE
                    )
                )
            )
        )
    }

    fun scoreAt(position: Position): Command {
        when (position) {
            Position.L2 -> return accept(SuperstructureRequest(SuperstructureStates.L2_SCORE))
            Position.L3 -> return accept(SuperstructureRequest(SuperstructureStates.L3_SCORE))
            Position.L4 -> return accept(SuperstructureRequest(SuperstructureStates.L4_SCORE))
            else -> throw IllegalArgumentException("Invalid position for scoring: $position")
        }
    }

    fun fixPivot(): Command {
        return accept(
            SuperstructureRequest(SuperstructureStates.FIX_PIVOT)
            // this is one of the rare cases where we don't want to move the climber
            // by way of directly proxying the command to the climber subsystem
            // like, seriously, it'll snap in half if it moves up even just an inch
        ).deadlineFor(climber.moveClimberOpenLoop({ 0.0 }, { 0.0 }).asProxy())
    }

    fun fixPivotAndScoreL4(): Command {
        return accept(SuperstructureStateRequest(Position.L4)).deadlineFor(
            climber.moveClimberOpenLoop(
                { 0.0 },
                { 0.0 }).asProxy()
        )
    }
}