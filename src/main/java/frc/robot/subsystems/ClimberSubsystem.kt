package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase


object ClimberSubsystem : SubsystemBase() {
    private var extended: Boolean = false
    // motor position constants are located in Constants

    // Set motor to lowered position
    fun disengage(): Command = runOnce {
        extended = false

        // TODO: Move motor to pos
    }

    // Set motor to raised positon
    fun engage(): Command = runOnce {
        extended = true

        // TODO: move motor to pos
    }



}