package frc.robot.util.superstructurecommands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.util.SuperstructureCommand
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.RobotContainer

class L2 : SuperstructureCommand {
    override val friendlyName: String = "L2"
    override val finalArmRestingAngle: Double = Constants.ArmConstants.ArmState.L2
    override val finalElevatorHeight: Double = Constants.ElevatorConstants.L2
    private var command: Command = Commands.none()

     fun construct(arm: Arm, elevator: Elevator): SuperstructureCommand {
        RobotContainer.statusTopic.set("L2")
        val sc = L2()
        sc.command = elevator.goToPosition(Constants.ElevatorConstants.ElevatorState.L2)
            .andThen(arm.moveArmToAngle(Constants.ArmConstants.ArmState.L2))
        return sc
    }

    override fun setCommand(command: Command) {
        this.command = command
    }
    override fun execute(): Command {
        return command
    }
}