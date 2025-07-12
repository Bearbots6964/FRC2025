package frc.robot.automation.superstructure

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.automation.Automator
import frc.robot.automation.Request
import frc.robot.automation.SubsystemData
import frc.robot.automation.superstructure.SuperstructureConstants.CLIMBER_POSITION_RETRACTED
import frc.robot.automation.superstructure.requests.IntakeState
import frc.robot.automation.superstructure.requests.SuperstructureRequest
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.arm.ClawIntake
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.elevator.Elevator

class SuperstructureAutomator : Automator {
    val arm: Arm
    val climber: Climber
    val elevator: Elevator
    val intake: ClawIntake
    override var running: Boolean = false
    val grabbed
        get() = intake.grabbed

    override fun accept(request: Request): Command {
        if (request !is SuperstructureRequest) return Commands.none()
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
        return Commands.sequence(
            Commands.runOnce({ running = true }), *list.toTypedArray()
        ).finallyDo(Runnable { running = false })
    }

    constructor(data: SubsystemData) {
        arm = data.arm
        climber = data.climber
        elevator = data.elevator
        intake = data.intake
    }

    fun moveArm(to: Double): Command = arm.moveArmToAngle(to).asProxy()

    fun moveClimber(to: Double): Command = climber.pivotToPosition(to).asProxy()

    fun moveElevator(to: Double): Command = elevator.goToPosition(to).asProxy()

    fun spinIntake(state: IntakeState): Command {
        return when (state) {
            IntakeState.INTAKE -> intake.intake().asProxy()
            IntakeState.ALGAE_INTAKE -> intake.intakeWithoutStoppingForAlgae().asProxy()
            IntakeState.OUTTAKE -> intake.outtake().asProxy()
            IntakeState.STOP -> intake.stop().asProxy()
        }
    }

    fun goToL1(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.L1.armPosition,
                elevatorPosition = SuperstructureConstants.L1.elevatorPosition,
            )
        )
    }

    fun goToL2(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.L2.armPosition,
                elevatorPosition = SuperstructureConstants.L2.elevatorPosition,
            )
        )
    }

    fun goToL3(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.L3.armPosition,
                elevatorPosition = SuperstructureConstants.L3.elevatorPosition,
            )
        )
    }

    fun goToL4(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.L4.armPosition,
                elevatorPosition = SuperstructureConstants.L4.elevatorPosition,
            )
        )
    }

    fun goToHome(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.HOME.armPosition,
                elevatorPosition = SuperstructureConstants.HOME.elevatorPosition,
            )
        )
    }

    fun goToPreCoralPickup(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.PRE_CORAL_PICKUP.armPosition,
                elevatorPosition = SuperstructureConstants.PRE_CORAL_PICKUP.elevatorPosition,
                climberPosition = SuperstructureConstants.CLIMBER_POSITION_RETRACTED,
            )
        )
    }

    fun goToCoralPickup(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.CORAL_PICKUP.armPosition,
                elevatorPosition = SuperstructureConstants.CORAL_PICKUP.elevatorPosition,
            )
        )
    }

    fun goToBargeLaunch(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.BARGE_LAUNCH.armPosition,
                elevatorPosition = SuperstructureConstants.BARGE_LAUNCH.elevatorPosition,
            )
        )
    }

    fun goToAlgaeIntake(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.ALGAE_INTAKE.armPosition,
                elevatorPosition = SuperstructureConstants.ALGAE_INTAKE.elevatorPosition,
            )
        )
    }

    fun goToUpperReefAlgae(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.UPPER_REEF_ALGAE.armPosition,
                elevatorPosition = SuperstructureConstants.UPPER_REEF_ALGAE.elevatorPosition,
            )
        )
    }

    fun goToLowerReefAlgae(): Command {
        return accept(
            SuperstructureRequest(
                armPosition = SuperstructureConstants.LOWER_REEF_ALGAE.armPosition,
                elevatorPosition = SuperstructureConstants.LOWER_REEF_ALGAE.elevatorPosition,
            )
        )
    }

    fun pickUpCoral(): Command {
        return accept(
            SuperstructureRequest( // pre-positioning
                armPosition = SuperstructureConstants.CORAL_PICKUP.armPosition,
                elevatorPosition = SuperstructureConstants.CORAL_PICKUP.elevatorPosition,
                climberPosition = SuperstructureConstants.CLIMBER_POSITION_EXTENDED,
            )
        ).andThen(
            accept(
                SuperstructureRequest( // spin intake
                    intakeSpeed = IntakeState.INTAKE
                )
            ).deadlineFor(
                accept(
                    SuperstructureRequest( // move climber inwards until secured
                        climberPosition = SuperstructureConstants.CLIMBER_POSITION_INTAKE
                    )
                )
            )
        )
    }
    fun scoreAt(position: Positions): Command {
        when (position) {
            Positions.L2 -> return accept(SuperstructureRequest(SuperstructureConstants.L2_SCORE))
            Positions.L3 -> return accept(SuperstructureRequest(SuperstructureConstants.L3_SCORE))
            Positions.L4 -> return accept(SuperstructureRequest(SuperstructureConstants.L4_SCORE))
            else -> throw IllegalArgumentException("Invalid position for scoring: $position")
        }
    }
}