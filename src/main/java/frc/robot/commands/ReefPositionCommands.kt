package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.Constants.ArmConstants
import frc.robot.Constants.ElevatorConstants.ElevatorState
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator

object ReefPositionCommands {
    var reefPosition = ElevatorState.L1
    fun l1(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L1")
        reefPosition = ElevatorState.L1
        return e.goToPosition(ElevatorState.L1)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L1))
    }
    fun l2(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L2")
        reefPosition = ElevatorState.L2
        return e.goToPosition(ElevatorState.L2)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L2))
    }
    fun l3(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L3")
        reefPosition = ElevatorState.L3
        return e.goToPosition(ElevatorState.L3)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L3))
    }
    fun l4(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L4")
        reefPosition = ElevatorState.L4
        return e.goToPosition(ElevatorState.L4)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L4))
    }
    fun coralStationPosition(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Coral Station Position")
        return e.goToPosition(Constants.ElevatorConstants.CORAL_PICKUP)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.CORAL_PICKUP))
    }
    fun home(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Home")
        return e.goToPosition(ElevatorState.HOME)
            .alongWith(a.moveArmToAngle(ArmConstants.ArmState.HOME))
    }

    fun goToPosition(e: Elevator, a: Arm, s: ElevatorState): Command {
        return when (s) {
            ElevatorState.L1 -> l1(e, a)
            ElevatorState.L2 -> l2(e, a)
            ElevatorState.L3 -> l3(e, a)
            ElevatorState.L4 -> l4(e, a)
            ElevatorState.CORAL_PICKUP -> coralStationPosition(e, a)
            ElevatorState.HOME -> home(e, a)
        }
    }
}