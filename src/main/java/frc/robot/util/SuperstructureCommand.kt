package frc.robot.util

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator

interface SuperstructureCommand {
    fun execute(): Command
    fun setCommand(command: Command)
    val friendlyName: String
    val finalArmRestingAngle: Double
    val finalElevatorHeight: Double
}