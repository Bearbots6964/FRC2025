package frc.robot.automation.superstructure

import frc.robot.Constants
import frc.robot.util.ShortString

// Constants for the superstructure states used in the robot's automation system.
// Elevator units are in rotations of the motor (1 inch = 3.6378272707 rotations).
// Arm units are in degrees, with 0 degrees being the arm extended straight out from the robot parallel to the floor.
object SuperstructureConstants {
    val L1 = SuperstructureState(
        elevatorPosition = 5.0,
        armPosition = 155.0
    )
    val L2  = SuperstructureState(
        elevatorPosition = 37.581,
        armPosition = -62.93
    )
    val L2_SCORE = SuperstructureState(
        elevatorPosition = 32.581, // -5.0
        armPosition = -36.93, // +26.0
    )
    val L3 = SuperstructureState(
        elevatorPosition = 107.838, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.L3
        armPosition = -67.85 // Constants.SuperstructureConstants.ArmConstants.ArmState.L3
    )
    val L3_SCORE = SuperstructureState(
        elevatorPosition = 102.838, // -5.0
        armPosition = -39.85 // +28.0
    )
    // Level 4 scoring position
    val L4 = SuperstructureState(
        elevatorPosition = 79.388, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.L4
        armPosition = 53.81 // Constants.SuperstructureConstants.ArmConstants.ArmState.L4
    )
    val L4_SCORE = SuperstructureState(
        elevatorPosition = 44.388, // -35.0
        armPosition = 40.81 // -13.0
    )
    // Home position
    val HOME = SuperstructureState(
        elevatorPosition = 45.3891363535, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.HOME
        armPosition = 215.5 // Constants.SuperstructureConstants.ArmConstants.ArmState.HOME
    )
    // Pre-coral pickup position
    val PRE_CORAL_PICKUP = SuperstructureState(
        elevatorPosition = 0.0, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.PRE_CORAL_PICKUP
        armPosition = 100.0 // Constants.SuperstructureConstants.ArmConstants.ArmState.PRE_CORAL_PICKUP
    )
    // Coral pickup position
    val CORAL_PICKUP = SuperstructureState(
        elevatorPosition = 42.781, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.CORAL_PICKUP
        armPosition = 218.125, // Constants.SuperstructureConstants.ArmConstants.ArmState.CORAL_PICKUP
        climberPosition = CLIMBER_POSITION_EXTENDED
    )
    // Barge launch position
    val BARGE_LAUNCH = SuperstructureState(
        elevatorPosition = 111.2, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.BARGE_LAUNCH
        armPosition = 77.0 // Constants.SuperstructureConstants.ArmConstants.ArmState.BARGE_LAUNCH
    )
    // Algae intake position
    val ALGAE_INTAKE = SuperstructureState(
        elevatorPosition = 0.0, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.ALGAE_INTAKE
        armPosition = -64.77 // Constants.SuperstructureConstants.ArmConstants.ArmState.ALGAE_INTAKE
    )
    // Upper reef algae position
    val UPPER_REEF_ALGAE = SuperstructureState(
        elevatorPosition = 0.0, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.UPPER_REEF_ALGAE
        armPosition = 25.0 // Constants.SuperstructureConstants.ArmConstants.ArmState.UPPER_REEF_ALGAE
    )
    // Lower reef algae position
    val LOWER_REEF_ALGAE = SuperstructureState(
        elevatorPosition = 7.276, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.LOWER_REEF_ALGAE
        armPosition = -6.8 // Constants.SuperstructureConstants.ArmConstants.ArmState.LOWER_REEF_ALGAE
    )
    const val CLIMBER_POSITION_EXTENDED = 40.0
    const val CLIMBER_POSITION_RETRACTED = 63.6
    const val CLIMBER_POSITION_INTAKE = 90.0
    const val INTAKE_SPEED = 0.4
    const val OUTTAKE_SPEED = 0.25
    const val ALGAE_INTAKE_SPEED = 0.45
}

data class SuperstructureState(
    val elevatorPosition: Double? = null,
    val armPosition: Double? = null,
    val climberPosition: Double? = null,
    val clawState: ClawState? = null,
)
enum class ClawState {
    INTAKING, NONE, OUTTAKING
}

enum class Positions : ShortString {
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
        return SuperstructureState(
            elevatorPosition = when (this) {
                HOME -> SuperstructureConstants.HOME.elevatorPosition
                L1 -> SuperstructureConstants.L1.elevatorPosition
                L2 -> SuperstructureConstants.L2.elevatorPosition
                L3 -> SuperstructureConstants.L3.elevatorPosition
                L4 -> SuperstructureConstants.L4.elevatorPosition
                PRE_CORAL_PICKUP -> SuperstructureConstants.PRE_CORAL_PICKUP.elevatorPosition
                CORAL_PICKUP -> SuperstructureConstants.CORAL_PICKUP.elevatorPosition
                BARGE_LAUNCH -> SuperstructureConstants.BARGE_LAUNCH.elevatorPosition
                ALGAE_INTAKE -> SuperstructureConstants.ALGAE_INTAKE.elevatorPosition
                UPPER_REEF_ALGAE -> SuperstructureConstants.UPPER_REEF_ALGAE.elevatorPosition
                LOWER_REEF_ALGAE -> SuperstructureConstants.LOWER_REEF_ALGAE.elevatorPosition
            },
            armPosition = when (this) {
                HOME -> SuperstructureConstants.HOME.armPosition
                L1 -> SuperstructureConstants.L1.armPosition
                L2 -> SuperstructureConstants.L2.armPosition
                L3 -> SuperstructureConstants.L3.armPosition
                L4 -> SuperstructureConstants.L4.armPosition
                PRE_CORAL_PICKUP -> SuperstructureConstants.PRE_CORAL_PICKUP.armPosition
                CORAL_PICKUP -> SuperstructureConstants.CORAL_PICKUP.armPosition
                BARGE_LAUNCH -> SuperstructureConstants.BARGE_LAUNCH.armPosition
                ALGAE_INTAKE -> SuperstructureConstants.ALGAE_INTAKE.armPosition
                UPPER_REEF_ALGAE -> SuperstructureConstants.UPPER_REEF_ALGAE.armPosition
                LOWER_REEF_ALGAE -> SuperstructureConstants.LOWER_REEF_ALGAE.armPosition
            }
        )
    }

    companion object {
        fun fromOldSpec(position: Constants.SuperstructureConstants.SuperstructureState): Positions {
            return when (position) {
                Constants.SuperstructureConstants.SuperstructureState.HOME -> HOME
                Constants.SuperstructureConstants.SuperstructureState.L1 -> L1
                Constants.SuperstructureConstants.SuperstructureState.L2 -> L2
                Constants.SuperstructureConstants.SuperstructureState.L3 -> L3
                Constants.SuperstructureConstants.SuperstructureState.L4 -> L4
                Constants.SuperstructureConstants.SuperstructureState.PRE_CORAL_PICKUP -> PRE_CORAL_PICKUP
                Constants.SuperstructureConstants.SuperstructureState.CORAL_PICKUP -> CORAL_PICKUP
                Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH -> BARGE_LAUNCH
                Constants.SuperstructureConstants.SuperstructureState.ALGAE_INTAKE -> ALGAE_INTAKE
                Constants.SuperstructureConstants.SuperstructureState.UPPER_REEF_ALGAE -> UPPER_REEF_ALGAE
                Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE -> LOWER_REEF_ALGAE
            }
        }
    }
}