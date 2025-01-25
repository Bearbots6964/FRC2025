// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.util

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.CANcoderSimState
import com.ctre.phoenix6.sim.TalonFXSimState
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import java.util.function.Supplier


object PhoenixUtil {
    /** Attempts to run the command until no error is produced.  */
    @JvmStatic
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
        for (i in 0..<maxAttempts) {
            val error = command.get()
            if (error.isOK) break
        }
    }

    @JvmStatic
    val simulationOdometryTimeStamps: DoubleArray
        get() {
            val odometryTimeStamps = DoubleArray(SimulatedArena.getSimulationSubTicksIn1Period())
            for (i in odometryTimeStamps.indices) {
                odometryTimeStamps[i] = (Timer.getFPGATimestamp()
                        - 0.02
                        + i * SimulatedArena.getSimulationDt().`in`(Units.Seconds))
            }

            return odometryTimeStamps
        }

    /**
     *
     *
     * <h2>Regulates the [SwerveModuleConstants] for a single module.</h2>
     *
     *
     * This method applies specific adjustments to the [SwerveModuleConstants] for simulation purposes. These
     * changes have no effect on real robot operations and address known simulation bugs:
     *
     *
     *  * **Inverted Drive Motors:** Prevents drive PID issues caused by inverted configurations.
     *  * **Non-zero CanCoder Offsets:** Fixes potential module state optimization issues.
     *  * **Steer Motor PID:** Adjusts PID values tuned for real robots to improve simulation
     * performance.
     *
     *
     * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants used on real
     * robot hardware.</h4>
     */
    @JvmStatic
    fun regulateModuleConstantForSimulation(
        moduleConstants: SwerveModuleConstants<*, *, *>
    ): SwerveModuleConstants<*, *, *> {
        // Skip regulation if running on a real robot
        if (RobotBase.isReal()) return moduleConstants

        // Apply simulation-specific adjustments to module constants
        return moduleConstants // Disable encoder offsets
            .withEncoderOffset(0.0) // Disable motor inversions for drive and steer motors
            .withDriveMotorInverted(false)
            .withSteerMotorInverted(false) // Disable CanCoder inversion
            .withEncoderInverted(false) // Adjust steer motor PID gains for simulation
            .withSteerMotorGains(
                moduleConstants
                    .SteerMotorGains
                    .withKP(70.0) // Proportional gain
                    .withKD(4.5)
            ) // Derivative gain
            // Adjust friction voltages
            .withDriveFrictionVoltage(Units.Volts.of(0.1))
            .withSteerFrictionVoltage(Units.Volts.of(0.15)) // Adjust steer inertia
            .withSteerInertia(Units.KilogramSquareMeters.of(0.05))
    }


    open class TalonFXMotorControllerSim(talonFX: TalonFX) : SimulatedMotorController {
        val id: Int

        private val talonFXSimState: TalonFXSimState

        init {
            this.id = instances++

            this.talonFXSimState = talonFX.simState
        }

        override fun updateControlSignal(
            mechanismAngle: Angle,
            mechanismVelocity: AngularVelocity,
            encoderAngle: Angle,
            encoderVelocity: AngularVelocity
        ): Voltage {
            talonFXSimState.setRawRotorPosition(encoderAngle)
            talonFXSimState.setRotorVelocity(encoderVelocity)
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage())
            return talonFXSimState.motorVoltageMeasure
        }

        companion object {
            private var instances = 0
        }
    }

    class TalonFXMotorControllerWithRemoteCancoderSim(talonFX: TalonFX, cancoder: CANcoder) :
        TalonFXMotorControllerSim(talonFX) {
        private val remoteCancoderSimState: CANcoderSimState = cancoder.simState

        override fun updateControlSignal(
            mechanismAngle: Angle,
            mechanismVelocity: AngularVelocity,
            encoderAngle: Angle,
            encoderVelocity: AngularVelocity
        ): Voltage {
            remoteCancoderSimState.setRawPosition(mechanismAngle)
            remoteCancoderSimState.setVelocity(mechanismVelocity)

            return super.updateControlSignal(
                mechanismAngle,
                mechanismVelocity,
                encoderAngle,
                encoderVelocity
            )
        }
    }
}
