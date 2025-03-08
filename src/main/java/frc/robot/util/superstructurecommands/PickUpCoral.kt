package frc.robot.util.superstructurecommands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.util.SuperstructureCommand

class PickUpCoral : SuperstructureCommand {
    override val friendlyName: String = "Pick Up Coral"
    override val finalArmRestingAngle: Double = 170.0
    override val finalElevatorHeight: Double = 25.0
    private var command: Command = Commands.none()

     fun construct(arm: Arm, elevator: Elevator): SuperstructureCommand {
        RobotContainer.statusTopic.set("Pick Up Coral")
        val sc = PickUpCoral()
        sc.command = elevator.goToPosition(60.0).andThen(
            arm.moveArmToAngle(12.0),
        ).andThen(
            elevator.goToPosition(30.0),
        ).andThen(
            elevator.goToPosition(45.0),
        ).andThen(
            arm.moveArmToAngle(25.0),
        ).andThen(
            arm.moveArmToAngle(170.0),
        )

        return sc
    }

    override fun setCommand(command: Command) {
       this.command = command
    }

    override fun execute(): Command {
        return command
    }
}