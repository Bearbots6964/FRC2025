package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


object ClimberSubsystem : SubsystemBase() {
    private var extended: Boolean = false
    private val motor: TalonFX = TalonFX(2, "rio")
    // motor position constants are located in Constants

    init {
        motor.setNeutralMode(NeutralModeValue.Brake)

    }

    // Set motor to lowered position
    fun disengage(): Command = runOnce {
        extended = false

        motor.setPosition(Units.Degrees.of(Constants.ClimberConstants.LOWERED_POS))
    }

    // Set motor to raised position
    fun engage(): Command = runOnce {
        extended = true

        motor.setPosition(Units.Degrees.of(Constants.ClimberConstants.EXTENDED_POS))
    }



}