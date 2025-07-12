package frc.robot.automation.superstructure.requests

import frc.robot.automation.Request
import frc.robot.automation.superstructure.ClawState
import frc.robot.automation.superstructure.SuperstructureState

class SuperstructureRequest : Request {
    val armPosition: Double?
    val elevatorPosition: Double?
    val climberPosition: Double?
    val intakeSpeed: IntakeState?

    constructor(
        armPosition: Double? = null,
        elevatorPosition: Double? = null,
        climberPosition: Double? = null,
        intakeSpeed: IntakeState? = null
    ) {
        this.armPosition = armPosition
        this.elevatorPosition = elevatorPosition
        this.climberPosition = climberPosition
        this.intakeSpeed = intakeSpeed
    }

    constructor(state: SuperstructureState) {
        this.armPosition = state.armPosition
        this.elevatorPosition = state.elevatorPosition
        this.climberPosition = state.climberPosition
        this.intakeSpeed = when (state.clawState) {
            ClawState.INTAKING -> IntakeState.INTAKE
            ClawState.OUTTAKING -> IntakeState.OUTTAKE
            ClawState.NONE -> IntakeState.STOP
            null -> throw IllegalStateException("ClawState is NULL")
        }
    }

}

enum class IntakeState {
    INTAKE,
    ALGAE_INTAKE,
    OUTTAKE,
    STOP
}