package frc.robot.automation.drivebase

object DrivebaseConstants {
}

/**
 * Constants for the Coral Station positioning system.
 * All units are in inches or degrees unless otherwise specified.
 */
object CoralStationPositioning {
    // Desired distance from the center of the robot to the Coral Station wall.
    const val PERPENDICULAR_DISTANCE_FROM_CORAL_STATION = 16.77
    // Desired nudge distance for lateral alignment with the Coral Station.
    // Good for aligning the robot to the very edge of the Coral Station wall.
    // Using the WPILib coordinate system, this would be the Y axis.
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    const val NUDGE_DISTANCE_FROM_CORAL_STATION = 17.0
}

/**
 * Constants for the Reef positioning system.
 * All units are in inches or degrees unless otherwise specified.
 */
object ReefPositioning {
    // Desired distance from the center of the robot to the Reef wall.
    const val PERPENDICULAR_DISTANCE_FROM_REEF = 16.77

    // Desired lateral distance for placing Coral on the left side of the Reef.
    const val LEFT_REEF_LATERAL_DISTANCE = 4.639
    // Desired lateral distance for placing Coral on the right side of the Reef.
    const val RIGHT_REEF_LATERAL_DISTANCE = 6.139
}

object PathfindingConstants {
    const val CORAL_INTAKE_SPEED = 0.6
    const val TO_REEF_SPEED = 0.7
    const val TO_BARGE_SPEED = 0.5
}