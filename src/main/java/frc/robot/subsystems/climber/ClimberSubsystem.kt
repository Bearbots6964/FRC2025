package frc.robot.subsystems.climber

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs


class ClimberSubsystem : SubsystemBase {

    private val io: ClimberIO;
    private val inputs: ClimberIOInputsAutoLogged = ClimberIOInputsAutoLogged()
    // motor position constants are located in Constants

    constructor(io: ClimberIO) {
        this.io = io;

    }

    override fun periodic() {
        io.updateInputs(inputs)
    }

    // Set motor to lowered position
    fun disengage(): Command = runOnce {
        io.setExtended(false)
    }

    // Set motor to raised position
    fun engage(): Command = runOnce {
        io.setExtended(true)
    }



}