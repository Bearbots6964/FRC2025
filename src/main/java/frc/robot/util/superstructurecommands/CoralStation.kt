package frc.robot.util.superstructurecommands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.util.SuperstructureCommand
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.RobotContainer

class CoralStation : SuperstructureCommand {
    override val friendlyName: String = "CoralStation"
    override val finalArmRestingAngle: Double = Constants.ArmConstants.ArmState.CORAL_PICKUP
    override val finalElevatorHeight: Double = Constants.ElevatorConstants.CORAL_PICKUP
    private var command: Command = Commands.none()
    var self: SuperstructureCommand = this

     fun construct(arm: Arm, elevator: Elevator): SuperstructureCommand {
        RobotContainer.statusTopic.set("CoralStation")
        val sc = CoralStation()
        sc.command = elevator.goToPosition(Constants.ElevatorConstants.CORAL_PICKUP)
            .alongWith(arm.moveArmToAngle(Constants.ArmConstants.ArmState.CORAL_PICKUP))
        return this
    }

    override fun setCommand(command: Command) {
        this.command = command
    }

    override fun execute(): Command {
        return command
    }
}