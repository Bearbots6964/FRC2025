package frc.robot.automation.drivebase

import frc.robot.SuperstructureStates
import frc.robot.automation.states.ShortString
import frc.robot.automation.superstructure.Position
import frc.robot.automation.superstructure.SuperstructureState

class DrivebaseAutomationTypes {
}

interface ReefLocation
enum class CoralPlacement : ReefLocation {
    A, B, C, D, E, F, G, H, I, J, K, L;

    /**
     * Converts this `CoralPlacement` to a `UnifiedReefLocation`.
     * This is useful for converting legacy coral placements to the unified reef locations.
     */
    fun toUnifiedReefLocation(): UnifiedReefLocation {
        return when (this) {
            A -> UnifiedReefLocation.A
            B -> UnifiedReefLocation.B
            C -> UnifiedReefLocation.C
            D -> UnifiedReefLocation.D
            E -> UnifiedReefLocation.E
            F -> UnifiedReefLocation.F
            G -> UnifiedReefLocation.G
            H -> UnifiedReefLocation.H
            I -> UnifiedReefLocation.I
            J -> UnifiedReefLocation.J
            K -> UnifiedReefLocation.K
            L -> UnifiedReefLocation.L
        }
    }
}

/**
 * List of coral pipes that are on the left side of their respective face.
 */
val leftPositions = listOf(
    CoralPlacement.A,
    CoralPlacement.C,
    CoralPlacement.E,
    CoralPlacement.G,
    CoralPlacement.I,
    CoralPlacement.K
)

/**
 * Possible algae placements on the reef.
 */
enum class AlgaePlacement : ReefLocation {
    AB, CD, EF, GH, IJ, KL
}

/**
 * Enum representing the two possible coral stations on the field.
 */
enum class CoralStation : ShortString {
    LEFT {
        override fun toString() = "Left"
        override fun toShortString(): String = "&L"
    },
    RIGHT {
        override fun toString() = "Right"
        override fun toShortString(): String = "&R"
    };
}

/**
 * The direction in which the robot should nudge to align with the coral station relative to the driver station wall.
 *
 * For example, if it is necessary to be as far away from the driver station wall as possible to allow multiple robots to fit in the station, one would use the `OUTWARD` direction.
 *
 * - INWARD: Nudge closer to the driver station wall.
 * - OUTWARD: Nudge farther from the driver station wall.
 * - NONE: No nudge; use the default position.
 */
enum class CoralStationNudgeDirection : ShortString {
    INWARD {
        override fun toShortString() = "Inward"
        override fun toString() = "\\/"
    },
    OUTWARD {
        override fun toShortString() = "Outward"
        override fun toString() = "/\\"
    },
    NONE {
        override fun toShortString() = "None"
        override fun toString() = "-"
    }
}

/**
 * Enum representing all possible reef locations for both coral and algae.
 */
enum class UnifiedReefLocation : ReefLocation, ShortString {
    A {
        override fun toString() = "A"
        override fun toShortString() = "#A"
    },
    B {
        override fun toString() = "B"
        override fun toShortString() = "#B"
    },
    C {
        override fun toString() = "C"
        override fun toShortString() = "#C"
    },
    D {
        override fun toString() = "D"
        override fun toShortString() = "#D"
    },
    E {
        override fun toString() = "E"
        override fun toShortString() = "#E"
    },
    F {
        override fun toString() = "F"
        override fun toShortString() = "#F"
    },
    G {
        override fun toString() = "G"
        override fun toShortString() = "#G"
    },
    H {
        override fun toString() = "H"
        override fun toShortString() = "#H"
    },
    I {
        override fun toString() = "I"
        override fun toShortString() = "#I"
    },
    J {
        override fun toString() = "J"
        override fun toShortString() = "#J"
    },
    K {
        override fun toString() = "K"
        override fun toShortString() = "#K"
    },
    L {
        override fun toString() = "L"
        override fun toShortString() = "#L"
    },
    AB {
        override fun toString() = "AB Algae"
        override fun toShortString() = "%AB"
    },
    CD {
        override fun toString() = "CD Algae"
        override fun toShortString() = "%CD"
    },
    EF {
        override fun toString() = "EF Algae"
        override fun toShortString() = "%EF"
    },
    GH {
        override fun toString() = "GH Algae"
        override fun toShortString() = "%GH"
    },
    IJ {
        override fun toString() = "IJ Algae"
        override fun toShortString() = "%IJ"
    },
    KL {
        override fun toString() = "KL Algae"
        override fun toShortString() = "%KL"
    },
    NONE {
        override fun toString() = "None"
        override fun toShortString() = "#_"
    };

    /**
     * Converts this `UnifiedReefLocation` to a `SuperstructureState`.
     */
    fun toAlgaeState(): SuperstructureState {
        return when (this) {
            AB, EF, IJ -> SuperstructureStates.UPPER_REEF_ALGAE
            CD, GH, KL -> SuperstructureStates.LOWER_REEF_ALGAE
            else -> throw IllegalArgumentException("Cannot convert $this to an algae state")
        }
    }

    /**
     * Converts this `UnifiedReefLocation` to a `Position` representing where the superstructure would be for this location.
     */
    fun toAlgaePosition(): Position {
        return when (this) {
            AB, EF, IJ -> Position.UPPER_REEF_ALGAE
            CD, GH, KL -> Position.LOWER_REEF_ALGAE

            else -> throw IllegalArgumentException("Cannot convert $this to an algae position")
        }
    }
}