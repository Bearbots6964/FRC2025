package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.Constants.SuperstructureConstants.ArmConstants
import frc.robot.Constants.SuperstructureConstants.ElevatorConstants.ElevatorState
import frc.robot.Constants.SuperstructureConstants.SuperstructureState
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.elevator.Elevator

object SuperstructureCommands {
    var reefPosition = SuperstructureState.L1
    fun l1(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L1")
        reefPosition = SuperstructureState.L1
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(SuperstructureState.L1)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.L1))
        ).withName("Superstructure to L1")
    }

    fun l2(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L2")
        reefPosition = SuperstructureState.L2
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(SuperstructureState.L2)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.L2))
        ).withName("Superstructure to L2")
    }

    fun l3(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L3")
        reefPosition = SuperstructureState.L3
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(SuperstructureState.L3)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.L3))
        ).withName("Superstructure to L3")
    }

    fun l4(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("L4")
        reefPosition = SuperstructureState.L4
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(SuperstructureState.L4)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.L4))
        ).withName("Superstructure to L4")
    }

    fun coralStationPosition(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Coral Station Position")
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(ElevatorState.CORAL_PICKUP)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.CORAL_PICKUP))
        ).withName("Superstructure to Coral Station Position")
    }

    fun home(e: Elevator, a: Arm, c: Climber): Command {
        RobotContainer.statusTopic.set("Home")
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(ElevatorState.HOME)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.HOME)).alongWith(c.moveClimberToCageCatchPosition())
                .andThen(c.moveClimberToIntakePosition())
        ).withName("Superstructure to Home Position")
    }

    fun preCoralPickup(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Pre Coral Pickup")
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(ElevatorState.PRE_CORAL_PICKUP)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.PRE_CORAL_PICKUP))
        ).withName("Superstructure to Pre Coral Pickup Position")
    }

    fun bargeLaunch(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Barge Launch")
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(ElevatorState.BARGE_LAUNCH)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.BARGE_LAUNCH))
        ).withName("Superstructure to Barge Algae Launch Position")
    }

    fun algaeIntake(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Algae Intake")
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(ElevatorState.ALGAE_INTAKE)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.ALGAE_INTAKE))
        ).withName("Superstructure to Algae Intake Position")
    }

    fun upperReefAlgae(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Upper Reef Algae")
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(ElevatorState.UPPER_REEF_ALGAE)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.UPPER_REEF_ALGAE))
        ).withName("Superstructure to Upper Reef Algae Position")
    }

    fun lowerReefAlgae(e: Elevator, a: Arm): Command {
        RobotContainer.statusTopic.set("Lower Reef Algae")
        return ensureSuperstructureSafety(e, a).andThen(
            e.goToPosition(ElevatorState.LOWER_REEF_ALGAE)
                .andThen(a.moveArmToAngle(ArmConstants.ArmState.LOWER_REEF_ALGAE))
        ).withName("Superstructure to Lower Reef Algae Position")
    }

    fun ensureSuperstructureSafety(e: Elevator, a: Arm): Command {
        return if (e.position < Constants.SuperstructureConstants.ElevatorConstants.armClearancePosition && a.armAngle < ArmConstants.safeAngle) a.moveArmToAngle(
            ArmConstants.safeAngle
        )
            .andThen(e.goToPosition(Constants.SuperstructureConstants.ElevatorConstants.armClearancePosition))
            .withName("Ensure Superstructure Safety")
        else Commands.none()
    }


    fun goToPosition(e: Elevator, a: Arm, c: Climber, s: SuperstructureState): Command {
        return when (s) {
            SuperstructureState.L1 -> l1(e, a)
            SuperstructureState.L2 -> l2(e, a)
            SuperstructureState.L3 -> l3(e, a)
            SuperstructureState.L4 -> l4(e, a)
            SuperstructureState.CORAL_PICKUP -> coralStationPosition(e, a)
            SuperstructureState.HOME -> home(e, a, c)
            SuperstructureState.PRE_CORAL_PICKUP -> preCoralPickup(e, a)
            SuperstructureState.BARGE_LAUNCH -> bargeLaunch(e, a)
            SuperstructureState.ALGAE_INTAKE -> algaeIntake(e, a)
            SuperstructureState.UPPER_REEF_ALGAE -> upperReefAlgae(e, a)
            SuperstructureState.LOWER_REEF_ALGAE -> lowerReefAlgae(e, a)
        }
    }
}