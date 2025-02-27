// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.DriveCommands
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.vision.*
import frc.robot.subsystems.vision.VisionConstants.*
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Subsystems
    private var drive: Drive
    private var vision: Vision

    private var driveSimulation: SwerveDriveSimulation? = null

    // Controller
    private val controller = CommandXboxController(0)

    // Dashboard inputs
    private val autoChooser: LoggedDashboardChooser<Command>

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        DriverStation.silenceJoystickConnectionWarning(true)
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                // Real robot, instantiate hardware IO implementations
                drive = Drive(
                    GyroIOPigeon2(),
                    ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                    ModuleIOTalonFXReal(TunerConstants.FrontRight),
                    ModuleIOTalonFXReal(TunerConstants.BackLeft),
                    ModuleIOTalonFXReal(TunerConstants.BackRight)
                ) { _: Pose2d? -> }
                this.vision = Vision(
                    drive,
                    VisionIOPhotonVision(camera0Name, VisionConstants.robotToCamera0),
                    VisionIOPhotonVision(camera1Name, robotToCamera1))
            }

            Constants.Mode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation =
                    SwerveDriveSimulation(Drive.mapleSimConfig, Pose2d(3.0, 3.0, Rotation2d()))
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)
                drive = Drive(
                    GyroIOSim(driveSimulation!!.gyroSimulation),
                    ModuleIOTalonFXSim(
                        TunerConstants.FrontLeft, driveSimulation!!.modules[0]
                    ),
                    ModuleIOTalonFXSim(
                        TunerConstants.FrontRight, driveSimulation!!.modules[1]
                    ),
                    ModuleIOTalonFXSim(
                        TunerConstants.BackLeft, driveSimulation!!.modules[2]
                    ),
                    ModuleIOTalonFXSim(
                        TunerConstants.BackRight, driveSimulation!!.modules[3]
                    )
                ) { robotPose: Pose2d? -> driveSimulation!!.setSimulationWorldPose(robotPose) }
                vision = Vision(
                    drive,
                    VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0
                    ) { driveSimulation!!.simulatedDriveTrainPose },
                    VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1
                    ) { driveSimulation!!.simulatedDriveTrainPose })
            }

            else -> {
                // Replayed robot, disable IO implementations
                drive = Drive(
                    object : GyroIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {}
                ) { _: Pose2d? -> }
                vision = Vision(drive, object : VisionIO {}, object : VisionIO {})
            }
        }
        // Set up auto routines
        autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive)
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        // Configure the button bindings
        configureButtonBindings()

        SmartDashboard.putData(drive.pathfindToLowerCoralStation())
        SmartDashboard.putData(drive.pathfindToUpperCoralStation())
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.defaultCommand = DriveCommands.joystickDrive(
            drive,
            { -controller.leftY },
            { -controller.leftX },
            { -controller.rightX })

        // Lock to 0° when A button is held
        controller.a().whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    { -controller.leftY },
                    { -controller.leftX },
                    { Rotation2d() })
            )

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce({ drive.stopWithX() }, drive))

        // Reset gyro / odometry
        val resetGyro = if (Constants.currentMode == Constants.Mode.SIM)
            Runnable {
                drive.pose = driveSimulation!!
                    .simulatedDriveTrainPose
            } // reset odometry to actual robot pose during simulation
        else
            Runnable { drive.pose = Pose2d(drive.pose.translation, Rotation2d()) } // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true))

        controller.b().and { drive.currentZone == Constants.Zone.LOWER_CORAL_STATION }
            .onTrue(Commands.run({DriveCommands.joystickDriveAtAngle(drive, { -controller.leftY }, { -controller.leftX }, { Rotation2d(
                Angle.ofRelativeUnits(30.0, Units.Degrees)) })}, drive))
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()

    fun resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return

        driveSimulation!!.setSimulationWorldPose(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }

    fun displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return

        Logger.recordOutput(
            "FieldSimulation/RobotPosition",
            driveSimulation!!.simulatedDriveTrainPose
        )
        Logger.recordOutput(
            "FieldSimulation/Coral", *SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
        )
        Logger.recordOutput(
            "FieldSimulation/Algae", *SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
        )
    }
}
