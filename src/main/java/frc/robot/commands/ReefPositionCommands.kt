package frc.robot.commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator

object ReefPositionCommands {
    fun l1(e: Elevator, a: Arm): SequentialCommandGroup = e.goToPosition(Constants.ElevatorConstants.ElevatorState.L1)
        .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L1))
    fun l2(e: Elevator, a: Arm): SequentialCommandGroup = e.goToPosition(Constants.ElevatorConstants.ElevatorState.L2)
        .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L2))
    fun l3(e: Elevator, a: Arm): SequentialCommandGroup = e.goToPosition(Constants.ElevatorConstants.ElevatorState.L3)
        .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L3))
    fun l4(e: Elevator, a: Arm): SequentialCommandGroup = e.goToPosition(Constants.ElevatorConstants.ElevatorState.L4)
        .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L4))
    // TODO@TheGamer1002: This is where I left off
    // fun coralStationPosition(e: Elevator, a: Arm): SequentialCommandGroup = e.goToPosition(Constants.ElevatorConstants)
}