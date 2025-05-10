package frc.robot.util

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
    }
}

interface ShortString {
    fun toShortString(): String
}

