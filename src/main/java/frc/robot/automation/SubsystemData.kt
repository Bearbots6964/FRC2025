package frc.robot.automation

import edu.wpi.first.math.geometry.Translation2d
import frc.robot.automation.states.State
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.arm.ClawIntake
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.elevator.Elevator
import java.util.function.Supplier

data class SubsystemData(
    val drivebase: Drive,
    val arm: Arm,
    val climber: Climber,
    val elevator: Elevator,
    val intake: ClawIntake,
    val nudge: Supplier<Translation2d>,
    val state: State
)
