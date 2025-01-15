package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase

object ArmSubsystem: SubsystemBase() {
    //TODO: May need a Min & Max Angle in Constants.kt

    var currentPosition: Angle
        get() = getCurrentPosition()
        set(value) = goToPosition(value)

    private fun getCurrentPosition(): Angle {
        return currentPosition
    }

    private fun goToPosition(value: Angle): Unit {
        //TODO: Move motor to position
    }
}