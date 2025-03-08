package frc.robot.util.superstructurecommands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.util.SuperstructureCommand
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.RobotContainer

class Home : SuperstructureCommand {
    override val friendlyName: String = "Home"
    override val finalArmRestingAngle: Double = 0.0
    override val finalElevatorHeight: Double = 0.0
    private var command: Command = Commands.none()
    var self: SuperstructureCommand = this

     fun construct(arm: Arm, elevator: Elevator): SuperstructureCommand {
        RobotContainer.statusTopic.set("Home")
        val sc = Home()
        sc.command = elevator.goToPosition(0.0)
            .andThen(arm.moveArmToAngle(0.0))
        return sc
    }

    override fun setCommand(command: Command) {
        this.command = command
    }

    override fun execute(): Command {
        return command
    }
}