package frc.robot.subsystems.climber

import frc.robot.Constants

object Climber {
    private val io = when (Constants.currentMode) {
        Constants.Mode.REAL -> Real
        Constants.Mode.SIM -> Sim
        Constants.Mode.REPLAY -> object : ClimberIo {}
    }
}