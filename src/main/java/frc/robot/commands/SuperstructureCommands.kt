package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.Constants.ArmConstants
import frc.robot.Constants.ElevatorConstants.ElevatorState
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator

object SuperstructureCommands {
    var reefPosition = ElevatorState.L1
    fun l1(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L1")
        reefPosition = ElevatorState.L1
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(ElevatorState.L1)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L1)))
    }
    fun l2(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L2")
        reefPosition = ElevatorState.L2
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(ElevatorState.L2)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L2)))
    }
    fun l3(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L3")
        reefPosition = ElevatorState.L3
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(ElevatorState.L3)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L3)))
    }
    fun l4(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L4")
        reefPosition = ElevatorState.L4
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(ElevatorState.L4)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.L4)))
    }
    fun coralStationPosition(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Coral Station Position")
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(Constants.ElevatorConstants.CORAL_PICKUP)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.CORAL_PICKUP)))
    }
    fun home(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Home")
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(Constants.ElevatorConstants.HOME)
            .alongWith(a.moveArmToAngle(ArmConstants.ArmState.HOME)))
    }
    fun preCoralPickup(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Pre Coral Pickup")
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(Constants.ElevatorConstants.PRE_CORAL_PICKUP)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.PRE_CORAL_PICKUP)))
    }
    fun bargeLaunch(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Barge Launch")
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(Constants.ElevatorConstants.BARGE_LAUNCH)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.BARGE_LAUNCH)))
    }
    fun algaeIntake(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Algae Intake")
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(Constants.ElevatorConstants.ALGAE_INTAKE)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.ALGAE_INTAKE)))
    }
    fun upperReefAlgae(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Upper Reef Algae")
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(Constants.ElevatorConstants.UPPER_REEF_ALGAE)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.UPPER_REEF_ALGAE)))
    }
    fun lowerReefAlgae(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Lower Reef Algae")
        return ensureSuperstructureSafety(e, a).andThen(e.goToPosition(Constants.ElevatorConstants.LOWER_REEF_ALGAE)
            .andThen(a.moveArmToAngle(ArmConstants.ArmState.LOWER_REEF_ALGAE)))
    }

    fun ensureSuperstructureSafety(e: Elevator, a: Arm): Command {
        return if (e.position < Constants.ElevatorConstants.safePosition && a.armAngle < ArmConstants.safeAngle)
            a.moveArmToAngle(ArmConstants.safeAngle).andThen(e.goToPosition(Constants.ElevatorConstants.safePosition))
        else Commands.none()
    }


    fun goToPosition(e: Elevator, a: Arm, s: ElevatorState): Command {
        return when (s) {
            ElevatorState.L1 -> l1(e, a)
            ElevatorState.L2 -> l2(e, a)
            ElevatorState.L3 -> l3(e, a)
            ElevatorState.L4 -> l4(e, a)
            ElevatorState.CORAL_PICKUP -> coralStationPosition(e, a)
            ElevatorState.HOME -> home(e, a)
            ElevatorState.PRE_CORAL_PICKUP -> preCoralPickup(e, a)
            ElevatorState.BARGE_LAUNCH -> bargeLaunch(e, a)
            ElevatorState.ALGAE_INTAKE -> algaeIntake(e, a)
            ElevatorState.UPPER_REEF_ALGAE -> upperReefAlgae(e, a)
            ElevatorState.LOWER_REEF_ALGAE -> lowerReefAlgae(e, a)
        }
    }
}