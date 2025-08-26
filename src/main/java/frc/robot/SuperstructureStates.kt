package frc.robot

import frc.robot.automation.superstructure.SuperstructureState

/**
 * Elevator height states.
 */
object SuperstructureStates {
    //const val rotationsPerInch = 3.6378272707
//    const val HOME = 45.3891363535
//    const val L1 = 5.0
//    const val L2 = 39.4 - 0.5 * rotationsPerInch
//    const val L3 = 104.2 + rotationsPerInch
//    const val L4 = 68.475 + (3 * rotationsPerInch) // TODO: Find actual value
//    const val PRE_CORAL_PICKUP = 0.0
//    const val CORAL_PICKUP = 44.6 - (0.5 * rotationsPerInch)
//    const val BARGE_LAUNCH = 111.2
//    const val ALGAE_INTAKE = 0.0
//    const val UPPER_REEF_ALGAE = 0.0
//    const val LOWER_REEF_ALGAE = 2.0 * rotationsPerInch

    @JvmStatic
    val L1 = SuperstructureState(
        elevatorPosition = 5.0, armPosition = 155.0
    )

    @JvmStatic
    val L2 = SuperstructureState(
        elevatorPosition = 37.581, armPosition = -62.93
    )

    @JvmStatic
    val L2_SCORE = SuperstructureState(
        elevatorPosition = 32.581, // -5.0
        armPosition = -36.93, // +26.0
    )

    @JvmStatic
    val L3 = SuperstructureState(
        elevatorPosition = 107.838, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.L3
        armPosition = -67.85 // Constants.SuperstructureConstants.ArmConstants.ArmState.L3
    )

    @JvmStatic
    val L3_SCORE = SuperstructureState(
        elevatorPosition = 102.838, // -5.0
        armPosition = -39.85 // +28.0
    )

    @JvmStatic
    // Level 4 scoring position
    val L4 = SuperstructureState(
        elevatorPosition = 79.388, armPosition = 53.81
    )

    @JvmStatic
    val L4_SCORE = SuperstructureState(
        elevatorPosition = 44.388, // -35.0
        armPosition = 40.81 // -13.0
    )

    @JvmStatic
    // Home position
    val HOME = SuperstructureState(
        elevatorPosition = 45.3891363535, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.HOME
        armPosition = 215.5 // Constants.SuperstructureConstants.ArmConstants.ArmState.HOME
    )

    @JvmStatic
    // Pre-coral pickup position
    val PRE_CORAL_PICKUP = SuperstructureState(
        elevatorPosition = 0.0, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.PRE_CORAL_PICKUP
        armPosition = 100.0 // Constants.SuperstructureConstants.ArmConstants.ArmState.PRE_CORAL_PICKUP
    )

    @JvmStatic
    // Coral pickup position
    val CORAL_PICKUP = SuperstructureState(
        elevatorPosition = 42.781, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.CORAL_PICKUP
        armPosition = 218.125, // Constants.SuperstructureConstants.ArmConstants.ArmState.CORAL_PICKUP
        climberPosition = CLIMBER_POSITION_EXTENDED
    )

    @JvmStatic
    // Barge launch position
    val BARGE_LAUNCH = SuperstructureState(
        elevatorPosition = 111.2, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.BARGE_LAUNCH
        armPosition = 77.0 // Constants.SuperstructureConstants.ArmConstants.ArmState.BARGE_LAUNCH
    )

    @JvmStatic
    // Algae intake position
    val ALGAE_INTAKE = SuperstructureState(
        elevatorPosition = 0.0, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.ALGAE_INTAKE
        armPosition = -64.77 // Constants.SuperstructureConstants.ArmConstants.ArmState.ALGAE_INTAKE
    )

    @JvmStatic
    // Upper reef algae position
    val UPPER_REEF_ALGAE = SuperstructureState(
        elevatorPosition = 0.0, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.UPPER_REEF_ALGAE
        armPosition = 25.0 // Constants.SuperstructureConstants.ArmConstants.ArmState.UPPER_REEF_ALGAE
    )

    @JvmStatic
    // Lower reef algae position
    val LOWER_REEF_ALGAE = SuperstructureState(
        elevatorPosition = 7.276, // Constants.SuperstructureConstants.ElevatorConstants.ElevatorState.LOWER_REEF_ALGAE
        armPosition = -6.8 // Constants.SuperstructureConstants.ArmConstants.ArmState.LOWER_REEF_ALGAE
    )

    @JvmStatic
    val FIX_PIVOT = SuperstructureState(
        elevatorPosition = 40.0, armPosition = PRE_CORAL_PICKUP.armPosition
    )

    const val CLIMBER_POSITION_EXTENDED = 40.0
    const val CLIMBER_POSITION_RETRACTED = 63.6
    const val CLIMBER_POSITION_INTAKE = 90.0
    const val INTAKE_SPEED = 0.4
    const val OUTTAKE_SPEED = 0.25
    const val ALGAE_INTAKE_SPEED = 0.45

}