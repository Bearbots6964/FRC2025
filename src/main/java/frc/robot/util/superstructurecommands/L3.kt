package frc.robot.util.superstructurecommands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.util.SuperstructureCommand
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.RobotContainer

class L3 : SuperstructureCommand {
    override val friendlyName: String = "L3"
    override val finalArmRestingAngle: Double = Constants.ArmConstants.ArmState.L3
    override val finalElevatorHeight: Double = Constants.ElevatorConstants.L3
    private var command: Command = Commands.none()

    fun construct(arm: Arm, elevator: Elevator): SuperstructureCommand {
        RobotContainer.statusTopic.set("L3")
        val sc = L3()
        sc.command = elevator.goToPosition(Constants.ElevatorConstants.ElevatorState.L3)
            .andThen(arm.moveArmToAngle(Constants.ArmConstants.ArmState.L3))
        return sc
    }

    override fun setCommand(command: Command) {
        this.command = command
    }

    override fun execute(): Command {
        return command
    }
}