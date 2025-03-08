package frc.robot.util.superstructurecommands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.util.SuperstructureCommand
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.RobotContainer

class L4 : SuperstructureCommand {
    override val friendlyName: String = "L4"
    override val finalArmRestingAngle: Double = Constants.ArmConstants.ArmState.L4
    override val finalElevatorHeight: Double = Constants.ElevatorConstants.L4
    private var command: Command = Commands.none()
    var self: SuperstructureCommand = this

     fun construct(arm: Arm, elevator: Elevator): SuperstructureCommand {
        RobotContainer.statusTopic.set("L4")
        val sc = L4()
        sc.command = elevator.goToPosition(Constants.ElevatorConstants.ElevatorState.L4)
            .andThen(arm.moveArmToAngle(Constants.ArmConstants.ArmState.L4))
        return sc
    }

    override fun setCommand(command: Command) {
        this.command = command
    }

    override fun execute(): Command {
        return command
    }
}