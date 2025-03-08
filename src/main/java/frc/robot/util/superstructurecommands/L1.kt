package frc.robot.util.superstructurecommands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.util.SuperstructureCommand

class L1 : SuperstructureCommand {
    override val friendlyName: String = "L1"
    override val finalArmRestingAngle: Double = Constants.ArmConstants.ArmState.L1
    override val finalElevatorHeight: Double = Constants.ElevatorConstants.L1
    private var command: Command = Commands.none()


    fun construct(arm: Arm, elevator: Elevator): SuperstructureCommand {
        RobotContainer.statusTopic.set("L1")
        val sc = L1()
        sc.command = elevator.goToPosition(Constants.ElevatorConstants.ElevatorState.L1)
            .andThen(arm.moveArmToAngle(Constants.ArmConstants.ArmState.L1))
        return sc
    }

    override fun setCommand(command: Command) {
        this.command = command
    }

    override fun execute(): Command {
        return command
    }
}