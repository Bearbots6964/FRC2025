package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator

object ReefPositionCommands {
    var reefPosition = Constants.ElevatorConstants.ElevatorState.L1
    fun l1(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L1")
        reefPosition = Constants.ElevatorConstants.ElevatorState.L1
        return e.goToPosition(Constants.ElevatorConstants.ElevatorState.L1)
            .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L1))
    }
    fun l2(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L2")
        reefPosition = Constants.ElevatorConstants.ElevatorState.L2
        return e.goToPosition(Constants.ElevatorConstants.ElevatorState.L2)
            .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L2))
    }
    fun l3(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L3")
        reefPosition = Constants.ElevatorConstants.ElevatorState.L3
        return e.goToPosition(Constants.ElevatorConstants.ElevatorState.L3)
            .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L3))
    }
    fun l4(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L4")
        reefPosition = Constants.ElevatorConstants.ElevatorState.L4
        return e.goToPosition(Constants.ElevatorConstants.ElevatorState.L4)
            .andThen(a.moveArmToAngle(Constants.ArmConstants.ArmState.L4))
    }
    fun coralStationPosition(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Coral Station Position")
        return e.goToPosition(Constants.ElevatorConstants.CORAL_PICKUP)
            .alongWith(a.moveArmToAngle(Constants.ArmConstants.ArmState.CORAL_PICKUP))
    }
}