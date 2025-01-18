package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.wpilibj2.command.SubsystemBase

object ArmSubsystem: SubsystemBase() {
    //TODO: May need a Min & Max Angle in Constants.kt

    //TODO: Get deviceId from armAxis Motor
    private val armAxisMotor: TalonFX = TalonFX(0)

    var currentPosition: Angle
        get() {
            return armAxisMotor.position.value
        }
        set(value) {
            //TODO: Move motor to position
        }
}