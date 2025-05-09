package frc.robot.util

enum class CoralStatus {
    NONE {
        override fun toString(): String {
            return "No Coral"
        }
    },
    ON_INTAKE {
        override fun toString(): String {
            return "Coral On Intake"
        }
    },
    IN_CLAW {
        override fun toString(): String {
            return "Coral In Claw"
        }
    }
}

enum class AlgaeStatus {
    NONE {
        override fun toString(): String {
            return "No Algae"
        }
    },
    IN_CLAW {
        override fun toString(): String {
            return "Algae In Claw"
        }
    }
}

enum class BargePosition {
    LEFT, MIDDLE, RIGHT, NONE
}

enum class AutoTask {
    TO_CORAL_STATION {
        override fun toString(): String {
            return "To Coral Station"
        }
    },
    TO_REEF {
        override fun toString(): String {
            return "To Reef"
        }
    },
    TO_BARGE {
        override fun toString(): String {
            return "To Barge"
        }
    },
    TO_CAGE {
        override fun toString(): String {
            return "To Cage"
        }
    },
    TO_ALGAE {
        override fun toString(): String {
            return "To Algae"
        }
    },
    IDLE {
        override fun toString(): String {
            return "Idle"
        }
    },
    WAITING {
        override fun toString(): String {
            return "Waiting for input..."
        }
    },
    SPIT_OUT_ALGAE {
        override fun toString(): String {
            return "Spit out algae"
        }
    }
}

