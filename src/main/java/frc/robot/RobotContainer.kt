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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.DriveCommands
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.vision.*
import frc.robot.subsystems.vision.VisionConstants.*
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Subsystems
    private var drive: Drive = when (Constants.currentMode) {
        Constants.Mode.REAL ->         // Real robot, instantiate hardware IO implementations
            Drive(
                GyroIOPigeon2(),
                ModuleIOTalonFX(TunerConstants.FrontLeft),
                ModuleIOTalonFX(TunerConstants.FrontRight),
                ModuleIOTalonFX(TunerConstants.BackLeft),
                ModuleIOTalonFX(TunerConstants.BackRight)
            )

        Constants.Mode.SIM ->         // Sim robot, instantiate physics sim IO implementations
            Drive(
                object : GyroIO {},
                ModuleIOSim(TunerConstants.FrontLeft),
                ModuleIOSim(TunerConstants.FrontRight),
                ModuleIOSim(TunerConstants.BackLeft),
                ModuleIOSim(TunerConstants.BackRight)
            )

        else ->         // Replayed robot, disable IO implementations
            Drive(
                object : GyroIO {},
                object : ModuleIO {},
                object : ModuleIO {},
                object : ModuleIO {},
                object : ModuleIO {})
    }

    private var vision: Vision = when (Constants.currentMode) {
        Constants.Mode.REAL ->         // Real robot, instantiate hardware IO implementations
            Vision(
                drive::addVisionMeasurement,
                VisionIOPhotonVision(camera0Name, robotToCamera0),
                VisionIOPhotonVision(camera1Name, robotToCamera1)
            )

        Constants.Mode.SIM ->
            // Sim robot, instantiate physics sim IO implementations
            Vision(
                drive::addVisionMeasurement,
                VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose)
            );

        else ->
            // Replayed robot, disable IO implementations
            // (Use same number of dummy implementations as the real robot)
            Vision(drive::addVisionMeasurement, object : VisionIO {}, object : VisionIO {});
    }

    // Controller
    private val controller =
        CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

    // Dashboard inputs
    private val autoChooser: LoggedDashboardChooser<Command>

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

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

        // Reset gyro to 0° when B button is pressed
        controller.b().onTrue(
                Commands.runOnce(
                    { drive.pose = Pose2d(drive.pose!!.translation, Rotation2d()) }, drive
                ).ignoringDisable(true)
            )
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()
}
