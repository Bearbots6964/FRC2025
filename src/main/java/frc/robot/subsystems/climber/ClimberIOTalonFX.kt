package frc.robot.subsystems.climber

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.Constants
import frc.robot.util.PhoenixUtil.tryUntilOk

/**
 * IO implementation for the climber using TalonFX (Falcon 500/Kraken).
 */
open class ClimberIOTalonFX(
) : ClimberIO {

    private val climberConfig: TalonFXConfiguration = Constants.ClimberConstants.config

    private val climberMotor = TalonFX(Constants.ClimberConstants.climberMotorID)

    private val climberVoltage: StatusSignal<Voltage>
    private val climberCurrent: StatusSignal<Current>

    private val climberConnectedDebouncer = Debouncer(0.5)

    private var isExtended = false

    init {
        // Apply configuration and reset position
        tryUntilOk(5) { climberMotor.configurator.apply(climberConfig, 0.25) }
        //tryUntilOk(5) { climberMotor.setPosition(Units.Degrees.of(0.0), 0.25) }

        // Status signals
        climberVoltage = climberMotor.motorVoltage
        climberCurrent = climberMotor.statorCurrent
    }

    override fun updateInputs(inputs: ClimberIO.ClimberIOInputs) {
        val climberStatus = BaseStatusSignal.refreshAll(climberCurrent, climberVoltage)

        inputs.extended = isExtended
        inputs.climberAppliedVolts = climberVoltage.value
        inputs.climberCurrent = climberCurrent.value
    }

    override fun setExtended(extended: Boolean) {
        isExtended = extended
        climberMotor.setPosition(if (extended) Constants.ClimberConstants.EXTENDED_POS else Constants.ClimberConstants.LOWERED_POS) // Assuming 1.0 extends and 0.0 retracts

    }

    override fun stop() {
        climberMotor.stopMotor()
    }

    /**
     * Helper method for simulation state.
     */
    fun getSimState(): TalonFXSimState = climberMotor.simState
}