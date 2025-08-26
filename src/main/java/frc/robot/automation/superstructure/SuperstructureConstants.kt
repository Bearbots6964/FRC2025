package frc.robot.automation.superstructure

import frc.robot.SuperstructureStates
import frc.robot.automation.states.ShortString

data class SuperstructureState(
    val elevatorPosition: Double? = null,
    val armPosition: Double? = null,
    val climberPosition: Double? = null,
    val clawState: ClawState? = null,
)
enum class ClawState {
    INTAKING, NONE, OUTTAKING
}

enum class Position : ShortString {
        HOME {
            override fun toString(): String = "Home"
            override fun toShortString(): String = "@_"
        },
        L1 {
            override fun toString(): String = "Level 1"
            override fun toShortString(): String = "@L1"
        },
        L2 {
            override fun toString(): String = "Level 2"
            override fun toShortString(): String = "@L2"
        },
        L3 {
            override fun toString(): String = "Level 3"
            override fun toShortString(): String = "@L3"
        },
        L4 {
            override fun toString(): String = "Level 4"
            override fun toShortString(): String = "@L4"
        },
        PRE_CORAL_PICKUP {
            override fun toString(): String = "Pre Coral Pickup"
            override fun toShortString(): String = "@PCP"
        },
        CORAL_PICKUP {
            override fun toString(): String = "Coral Pickup"
            override fun toShortString(): String = "@CP"
        },
        BARGE_LAUNCH {
            override fun toString(): String = "Barge Algae Launch"
            override fun toShortString(): String = "@AL"
        },
        ALGAE_INTAKE {
            override fun toString(): String = "Front Algae Intake"
            override fun toShortString(): String = "@AI"
        },
        UPPER_REEF_ALGAE {
            override fun toString(): String = "Upper Reef Algae"
            override fun toShortString(): String = "@UA"
        },
        LOWER_REEF_ALGAE {
            override fun toString(): String = "Lower Reef Algae"
            override fun toShortString(): String = "@LA"
        };

    fun toState(): SuperstructureState {
        return when (this) {
            HOME -> SuperstructureStates.HOME
            L1 -> SuperstructureStates.L1
            L2 -> SuperstructureStates.L2_SCORE
            L3 -> SuperstructureStates.L3_SCORE
            L4 -> SuperstructureStates.L4_SCORE
            PRE_CORAL_PICKUP -> SuperstructureStates.PRE_CORAL_PICKUP
            CORAL_PICKUP -> SuperstructureStates.CORAL_PICKUP
            BARGE_LAUNCH -> SuperstructureStates.BARGE_LAUNCH
            ALGAE_INTAKE -> SuperstructureStates.ALGAE_INTAKE
            UPPER_REEF_ALGAE -> SuperstructureStates.UPPER_REEF_ALGAE
            LOWER_REEF_ALGAE -> SuperstructureStates.LOWER_REEF_ALGAE
        }
    }
}