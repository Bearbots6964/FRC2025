package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.Constants.SuperstructureConstants.ArmConstants
import frc.robot.Constants.SuperstructureConstants.ElevatorConstants.ElevatorState
import frc.robot.Constants.SuperstructureConstants.SuperstructureState
import frc.robot.Robot
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.arm.ClawIntake
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.elevator.Elevator
import java.util.function.Consumer

class SuperstructureCommands(val stateConsumer: Consumer<SuperstructureState>) {
    fun l1(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.L1) }).andThen(
            e.goToPosition(SuperstructureState.L1)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.L1))
        ).withName("Superstructure to L1")
    }

    fun l2(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.L2) }).andThen(
            e.goToPosition(SuperstructureState.L2)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.L2))
        ).withName("Superstructure to L2")
    }

    fun l3(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.L3) }).andThen(
            e.goToPosition(SuperstructureState.L3)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.L3))
        ).withName("Superstructure to L3")
    }

    fun l4(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.L4) }).andThen(
            e.goToPosition(SuperstructureState.L4)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.L4))
        ).withName("Superstructure to L4")
    }

    fun l4WithoutSafety(e: Elevator, a: Arm): Command {
        return e.goToPosition(SuperstructureState.L4)
            .alongWith(a.moveArmToAngle(ArmConstants.ArmState.L4))
            .deadlineFor(Commands.runOnce({ stateConsumer.accept(SuperstructureState.L4) }))
            .withName("Superstructure to L4")
    }

    fun coralStationPosition(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.CORAL_PICKUP) })
            .andThen(
                e.goToPosition(ElevatorState.CORAL_PICKUP).alongWith(c.moveClimberToCageCatchPosition())
                    .alongWith(a.moveArmToAngle(ArmConstants.ArmState.CORAL_PICKUP))
            ).withName("Superstructure to Coral Station Position")
    }

    fun home(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.HOME) }).andThen(
            e.goToPosition(ElevatorState.HOME).alongWith(c.moveClimberToCageCatchPosition())
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.HOME))
        ).withName("Superstructure to Home Position")
    }

    fun homeWithoutSafety(e: Elevator, a: Arm, c: Climber): Command {
        return e.goToPosition(ElevatorState.HOME).alongWith(c.moveClimberToCageCatchPosition())
            .alongWith(a.moveArmToAngle(ArmConstants.ArmState.HOME))
            .andThen({ stateConsumer.accept(SuperstructureState.HOME) })
            .withName("Superstructure to Home Position")
    }

    fun preCoralPickup(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(
            e,
            a,
            c
        ).andThen({ stateConsumer.accept(SuperstructureState.PRE_CORAL_PICKUP) }).andThen(
            e.goToPosition(ElevatorState.PRE_CORAL_PICKUP)
                .alongWith(c.moveClimberToIntakePosition())
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.PRE_CORAL_PICKUP))
        ).withName("Superstructure to Pre Coral Pickup Position")
    }

    fun preCoralPickupWithoutSafety(e: Elevator, a: Arm): Command {
        return Commands.runOnce({ stateConsumer.accept(SuperstructureState.PRE_CORAL_PICKUP) }).andThen(
            e.goToPosition(ElevatorState.PRE_CORAL_PICKUP)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.PRE_CORAL_PICKUP))
        ).withName("Superstructure to Pre Coral Pickup Position")
    }
    fun bargeLaunch(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.BARGE_LAUNCH) })
            .andThen(
                e.goToPosition(ElevatorState.BARGE_LAUNCH)
                    .alongWith(a.moveArmToAngle(ArmConstants.ArmState.BARGE_LAUNCH))
            ).withName("Superstructure to Barge Algae Launch Position")
    }

    fun algaeIntake(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen({ stateConsumer.accept(SuperstructureState.ALGAE_INTAKE) })
            .andThen(
                e.goToPosition(ElevatorState.ALGAE_INTAKE)
                    .alongWith(a.moveArmToAngle(ArmConstants.ArmState.ALGAE_INTAKE))
            ).withName("Superstructure to Algae Intake Position")
    }

    fun algaeIntakeWithoutSafety(e: Elevator, a: Arm, c: Climber): Command {
        return e.goToPosition(ElevatorState.ALGAE_INTAKE)
            .alongWith(a.moveArmToAngle(ArmConstants.ArmState.ALGAE_INTAKE))
            .andThen({ stateConsumer.accept(SuperstructureState.ALGAE_INTAKE) })
            .withName("Superstructure to Algae Intake Position")
    }

    fun upperReefAlgae(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen(
            e.goToPosition(ElevatorState.UPPER_REEF_ALGAE)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.UPPER_REEF_ALGAE))
                .andThen({ stateConsumer.accept(SuperstructureState.UPPER_REEF_ALGAE) })
        ).withName("Superstructure to Upper Reef Algae Position")
    }

    fun lowerReefAlgae(e: Elevator, a: Arm, c: Climber): Command {
        return ensureSuperstructureSafety(e, a, c).andThen(
            e.goToPosition(ElevatorState.LOWER_REEF_ALGAE)
                .alongWith(a.moveArmToAngle(ArmConstants.ArmState.LOWER_REEF_ALGAE))
                .andThen({ stateConsumer.accept(SuperstructureState.LOWER_REEF_ALGAE) })
        ).withName("Superstructure to Lower Reef Algae Position")
    }

    fun ensureSuperstructureSafety(e: Elevator, a: Arm, c: Climber): Command {
        return if ((e.position < Constants.SuperstructureConstants.ElevatorConstants.armClearancePosition) && (a.armAngle < ArmConstants.safeAngle)) a.moveArmToAngle(
            ArmConstants.safeAngle
        ).withName("Ensure Superstructure Safety")
        else if ((e.position < 20.0) && (a.armAngle > 200.0) && (c.position > -30.0)) c.moveClimberToCageCatchPosition()
            .withName("Ensure Superstructure Safety")
        else Commands.none()
    }


    fun goToPosition(e: Elevator, a: Arm, c: Climber, s: SuperstructureState): Command {
        return when (s) {
            SuperstructureState.L1 -> l1(e, a, c)
            SuperstructureState.L2 -> l2(e, a, c)
            SuperstructureState.L3 -> l3(e, a, c)
            SuperstructureState.L4 -> l4(e, a, c)
            SuperstructureState.CORAL_PICKUP -> coralStationPosition(e, a, c)
            SuperstructureState.HOME -> home(e, a, c)
            SuperstructureState.PRE_CORAL_PICKUP -> preCoralPickup(e, a, c)
            SuperstructureState.BARGE_LAUNCH -> bargeLaunch(e, a, c)
            SuperstructureState.ALGAE_INTAKE -> algaeIntake(e, a, c)
            SuperstructureState.UPPER_REEF_ALGAE -> upperReefAlgae(e, a, c)
            SuperstructureState.LOWER_REEF_ALGAE -> lowerReefAlgae(e, a, c)
        }
    }

    fun goToPositionWithoutSafety(
        e: Elevator, a: Arm, c: Climber, s: SuperstructureState
    ): Command {
        return when (s) {
            SuperstructureState.L1 -> l1(e, a, c)
            SuperstructureState.L2 -> l2(e, a, c)
            SuperstructureState.L3 -> l3(e, a, c)
            SuperstructureState.L4 -> l4WithoutSafety(e, a)
            SuperstructureState.CORAL_PICKUP -> coralStationPosition(e, a, c)
            SuperstructureState.HOME -> home(e, a, c)
            SuperstructureState.PRE_CORAL_PICKUP -> preCoralPickup(e, a, c)
            SuperstructureState.BARGE_LAUNCH -> bargeLaunch(e, a, c)
            SuperstructureState.ALGAE_INTAKE -> algaeIntake(e, a, c)
            SuperstructureState.UPPER_REEF_ALGAE -> upperReefAlgae(e, a, c)
            SuperstructureState.LOWER_REEF_ALGAE -> lowerReefAlgae(e, a, c)
        }
    }

    fun scoreAtPosition(
        e: Elevator, a: Arm, f: ClawIntake, d: Drive, s: SuperstructureState
    ): Command {
        return when (s) {
            SuperstructureState.L1 -> Commands.runOnce({ Robot.reportError("Cannot score at L1") })
            SuperstructureState.L2 -> e.goToPositionDelta(-5.0).alongWith(
                a.moveArmAngleDelta(26.0)
            ).alongWith(
                f.outtakeFaster()
            ).withDeadline(
                Commands.waitSeconds(0.5).andThen(d.backUp())
            ).withName("Score L2")

            SuperstructureState.L3 -> e.goToPositionDelta(-5.0).alongWith(
                a.moveArmAngleDelta(28.0)
            ).alongWith(
                f.outtakeFaster()
            ).withDeadline(
                Commands.waitSeconds(0.5).andThen(d.backUp())
            ).withName("Score L3")

            SuperstructureState.L4 -> e.goToPositionDelta(-35.0).alongWith(
                f.outtake()
            ).alongWith(
                a.moveArmAngleDelta(-13.0)
            ).withDeadline(
                Commands.waitSeconds(0.5).andThen(d.backUp())
            ).withName("Score L4")

            else -> Commands.none()
        }
    }

    fun scoreAtPositionFaster(
        e: Elevator, a: Arm, f: ClawIntake, d: Drive, s: SuperstructureState
    ): Command {
        return when (s) {
            SuperstructureState.L1 -> Commands.runOnce({ Robot.reportError("Cannot score at L1") })
            SuperstructureState.L2 -> e.goToPositionDelta(-5.0).alongWith(
                a.moveArmAngleDelta(26.0)
            ).alongWith(
                f.outtakeFaster()
            ).withDeadline(
                Commands.waitSeconds(0.5).andThen(d.backUpFaster())
            ).withName("Score L2")

            SuperstructureState.L3 -> e.goToPositionDelta(-5.0).alongWith(
                a.moveArmAngleDelta(28.0)
            ).alongWith(
                f.outtakeFaster()
            ).withDeadline(
                Commands.waitSeconds(0.5).andThen(d.backUpFaster())
            ).withName("Score L3")

            SuperstructureState.L4 -> e.goToPositionDelta(-35.0).alongWith(
                f.outtakeFaster()
            ).alongWith(
                a.moveArmAngleDelta(-13.0)
            ).withDeadline(
                Commands.waitSeconds(0.5).andThen(d.backUpFaster())
            ).withName("Score L4")

            else -> Commands.none()
        }
    }

    fun scoreAtPositionWithoutDrive(
        e: Elevator, a: Arm, f: ClawIntake, s: SuperstructureState
    ): Command {
        return when (s) {
            SuperstructureState.L1 -> Commands.runOnce({ Robot.reportError("Cannot score at L1") })
            SuperstructureState.L2 -> e.goToPositionDelta(-5.0).alongWith(
                a.moveArmAngleDelta(20.0)
            ).alongWith(
                f.outtakeFaster()
            ).withName("Score L2")

            SuperstructureState.L3 -> e.goToPositionDelta(-5.0).alongWith(
                a.moveArmAngleDelta(25.0)
            ).alongWith(
                f.outtakeFaster()
            ).withName("Score L3")

            SuperstructureState.L4 -> e.goToPositionDelta(-35.0).alongWith(
                f.outtake()
            ).alongWith(
                a.moveArmAngleDelta(-10.0)
            ).withName("Score L4")

            else -> Commands.none()
        }
    }

    fun pickUpCoral(e: Elevator, a: Arm, f: ClawIntake, c: Climber): Command {
        return Commands.sequence(
            ensureSuperstructureSafety(e, a, c),
            Commands.parallel(
                e.goToPosition(ElevatorState.CORAL_PICKUP),
                a.moveArmToAngle(ArmConstants.ArmState.CORAL_PICKUP).until { a.armAngle > 200.0 },
                c.moveClimberToCageCatchPosition(),
                Commands.runOnce({ stateConsumer.accept(SuperstructureState.CORAL_PICKUP) })
            ),
            f.intake().deadlineFor(c.pivotToPosition(90.0)),
        ).withName("Pick Up Coral")
    }

}