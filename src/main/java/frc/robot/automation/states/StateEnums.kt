package frc.robot.automation.states

import frc.robot.util.CoralStatus as OtherCoralStatus
import frc.robot.util.AlgaeStatus as OtherAlgaeStatus
import frc.robot.util.BargePosition as OtherBargePosition
import frc.robot.util.CagePosition as OtherCagePosition
import frc.robot.util.AutoTask as OtherAutoTask

enum class CoralStatus : ShortString {
    NONE {
        override fun toString(): String = "No Coral"
        override fun toShortString(): String = "~_"
    },
    ON_INTAKE {
        override fun toString(): String = "Coral On Intake"
        override fun toShortString(): String = "~I"
    },
    IN_CLAW {
        override fun toString(): String = "Coral In Claw"
        override fun toShortString(): String = "~C"
    };
    companion object {
        fun fromOldSpec(status: OtherCoralStatus): frc.robot.automation.states.CoralStatus {
            return when (status) {
                OtherCoralStatus.NONE -> NONE
                OtherCoralStatus.ON_INTAKE -> ON_INTAKE
                OtherCoralStatus.IN_CLAW -> IN_CLAW
            }
        }
    }
}

enum class AlgaeStatus : ShortString {
    NONE {
        override fun toString(): String = "No Algae"
        override fun toShortString(): String = "•_"
    },
    IN_CLAW {
        override fun toString(): String = "Algae In Claw"
        override fun toShortString(): String = "•C"
    };
    companion object {
        fun fromOldSpec(status: OtherAlgaeStatus): AlgaeStatus {
            return when (status) {
                OtherAlgaeStatus.NONE -> NONE
                OtherAlgaeStatus.IN_CLAW -> IN_CLAW
            }
        }
    }
}

enum class BargePosition : ShortString {
    LEFT {
        override fun toShortString(): String = "^L"
        override fun toString(): String = "Left Barge"

    }, MIDDLE {
        override fun toShortString(): String = "^M"
        override fun toString(): String = "Middle Barge"
    }, RIGHT {
        override fun toShortString(): String = "^R"
        override fun toString(): String = "Right Barge"
    }, NONE {
        override fun toShortString(): String = "^_"
        override fun toString(): String = "No Barge"
    };
    companion object {
        fun fromOldSpec(position: OtherBargePosition): BargePosition {
            return when (position) {
                OtherBargePosition.LEFT -> LEFT
                OtherBargePosition.MIDDLE -> MIDDLE
                OtherBargePosition.RIGHT -> RIGHT
                OtherBargePosition.NONE -> NONE
            }
        }
    }
}

enum class CagePosition : ShortString {
    LEFT {
        override fun toShortString(): String = "*L"
        override fun toString(): String = "Left Cage"

    }, MIDDLE {
        override fun toShortString(): String = "*M"
        override fun toString(): String = "Middle Cage"
    }, RIGHT {
        override fun toShortString(): String = "*R"
        override fun toString(): String = "Right Cage"
    }, NONE {
        override fun toShortString(): String = "*_"
        override fun toString(): String = "No Cage"
    };
    companion object {
        fun fromOldSpec(position: OtherCagePosition): CagePosition {
            return when (position) {
                OtherCagePosition.LEFT -> LEFT
                OtherCagePosition.MIDDLE -> MIDDLE
                OtherCagePosition.RIGHT -> RIGHT
                OtherCagePosition.NONE -> NONE
            }
        }
    }
}

enum class AutoTask : ShortString {
    TO_CORAL_STATION {
        override fun toString(): String = "To Coral Station"
        override fun toShortString(): String = ">CS"
    },
    TO_REEF {
        override fun toString(): String = "To Reef"
        override fun toShortString(): String = ">R"
    },
    TO_BARGE {
        override fun toString(): String = "To Barge"
        override fun toShortString(): String = ">B"
    },
    TO_CAGE {
        override fun toString(): String = "To Cage"
        override fun toShortString(): String = ">C"
    },
    TO_ALGAE {
        override fun toString(): String = "To Algae"
        override fun toShortString(): String = ">A"
    },
    IDLE {
        override fun toString(): String = "Idle"
        override fun toShortString(): String = ">_"
    },
    WAITING {
        override fun toString(): String = "Waiting for input..."
        override fun toShortString(): String = ">..."
    },
    SPIT_OUT_ALGAE {
        override fun toString(): String = "Spit out algae"
        override fun toShortString(): String = ">SA"
    };
    companion object {
        fun fromOldSpec(task: OtherAutoTask): AutoTask {
            return when (task) {
                OtherAutoTask.TO_CORAL_STATION -> TO_CORAL_STATION
                OtherAutoTask.TO_REEF -> TO_REEF
                OtherAutoTask.TO_BARGE -> TO_BARGE
                OtherAutoTask.TO_CAGE -> TO_CAGE
                OtherAutoTask.TO_ALGAE -> TO_ALGAE
                OtherAutoTask.IDLE -> IDLE
                OtherAutoTask.WAITING -> WAITING
                OtherAutoTask.SPIT_OUT_ALGAE -> SPIT_OUT_ALGAE
            }
        }
    }
}

interface ShortString {
    fun toShortString(): String
}

