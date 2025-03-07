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
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants.VisionConstants.robotToCamera0
import frc.robot.Constants.VisionConstants.robotToCamera1
import frc.robot.commands.DriveCommands
import frc.robot.commands.DriveToNearestCoralStationCommand
import frc.robot.commands.DriveToNearestReefSideCommand
import frc.robot.commands.ReefPositionCommands
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.arm.ArmIO
import frc.robot.subsystems.arm.ArmIOTalonFX
import frc.robot.subsystems.arm.ArmIOTalonFXSim
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorIO
import frc.robot.subsystems.elevator.ElevatorIOTalonFX
import frc.robot.subsystems.vision.*
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
    private var arm: Arm
    private var elevator: Elevator

    private var driveSimulation: SwerveDriveSimulation? = null

    // Controller
    private val driveController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)
    private val markIVController = CommandGenericHID(2)
    private val buttonMacroController: CommandJoystick = CommandJoystick(3)

    // Dashboard inputs
    private val autoChooser: LoggedDashboardChooser<Command>

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
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
                    VisionIOPhotonVision(Constants.VisionConstants.camera0Name, robotToCamera0),
                    VisionIOPhotonVision(Constants.VisionConstants.camera1Name, robotToCamera1)
                )
                arm = Arm(
                    ArmIOTalonFX(
                        //TalonFXConfiguration(),
                        SparkMaxConfig(),
                    )
                )

                elevator = Elevator(
                    ElevatorIOTalonFX(
                        Constants.ElevatorConstants.leftMotorConfig,
                        Constants.ElevatorConstants.rightMotorConfig
                    )
                )
            }

            Constants.Mode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation =
                    SwerveDriveSimulation(Drive.mapleSimConfig, Pose2d(3.0, 3.0, Rotation2d()))
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)
                drive = Drive(
                    GyroIOSim(driveSimulation!!.gyroSimulation), ModuleIOTalonFXSim(
                        TunerConstants.FrontLeft, driveSimulation!!.modules[0]
                    ), ModuleIOTalonFXSim(
                        TunerConstants.FrontRight, driveSimulation!!.modules[1]
                    ), ModuleIOTalonFXSim(
                        TunerConstants.BackLeft, driveSimulation!!.modules[2]
                    ), ModuleIOTalonFXSim(
                        TunerConstants.BackRight, driveSimulation!!.modules[3]
                    )
                ) { robotPose: Pose2d? -> driveSimulation!!.setSimulationWorldPose(robotPose) }
                vision = Vision(
                    drive, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.camera0Name, robotToCamera0
                    ) { driveSimulation!!.simulatedDriveTrainPose }, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.camera1Name, robotToCamera1
                    ) { driveSimulation!!.simulatedDriveTrainPose })
                arm = Arm(
                    ArmIOTalonFXSim(

                    )
                )
                elevator = Elevator(object : ElevatorIO {})
            }

            else -> {
                // Replayed robot, disable IO implementations
                drive = Drive(
                    object : GyroIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {}) { _: Pose2d? -> }
                vision = Vision(drive, object : VisionIO {}, object : VisionIO {})

                elevator = Elevator(object : ElevatorIO {})
                arm = Arm(object : ArmIO {})
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
        autoChooser.addOption(
            "Arm SysId (Quasistatic Forward)", arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Arm SysId (Quasistatic Reverse)", arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Arm SysId (Dynamic Forward)", arm.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Arm SysId (Dynamic Reverse)", arm.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        autoChooser.addOption(
            "Elevator SysId (Quasistatic Forward)",
            elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )

        autoChooser.addOption(
            "Elevator SysId (Quasistatic Reverse)",
            elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )

        autoChooser.addOption(
            "Elevator SysId (Dynamic Forward)",
            elevator.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )

        autoChooser.addOption(
            "Elevator SysId (Dynamic Reverse)",
            elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse)
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
            { -driveController.leftY },
            { -driveController.leftX },
            { -driveController.rightX })

        // Lock to 0° when A button is held
        driveController.a().whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                { -driveController.leftY },
                { -driveController.leftX },
                { Rotation2d() })
        )

        // Switch to X pattern when X button is pressed
        driveController.x().onTrue(Commands.runOnce({ drive.stopWithX() }, drive))

        driveController.y().and(driveController.leftBumper())
            .onTrue(DriveToNearestReefSideCommand(drive, true).command)
        driveController.y().and(driveController.leftBumper().negate())
            .onTrue(DriveToNearestReefSideCommand(drive, false).command)
        driveController.rightBumper().onTrue(DriveToNearestCoralStationCommand(drive).command)

        // Reset gyro / odometry
        val resetGyro = if (Constants.currentMode == Constants.Mode.SIM) Runnable {
            drive.pose = driveSimulation!!.simulatedDriveTrainPose
        } // reset odometry to actual robot pose during simulation
        else Runnable { drive.pose = Pose2d(drive.pose.translation, Rotation2d()) } // zero gyro
        driveController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true))

        elevator.defaultCommand = elevator.stop()
        arm.defaultCommand = arm.stop()

        operatorController.a()
            .whileTrue(elevator.goToPosition(Constants.ElevatorConstants.ElevatorState.HOME))


        operatorController.b()
            .whileTrue(elevator.goToPosition(10.0).alongWith(arm.moveArmToAngle(150.0)))
        operatorController.x()
            .whileTrue(elevator.goToPosition(40.0).alongWith(arm.moveArmToAngle(150.0)))
        operatorController.y()
            .whileTrue(elevator.goToPosition(60.0).alongWith(arm.moveArmToAngle(170.0)))
        operatorController.rightTrigger().whileTrue(
            elevator.goToPosition(60.0).andThen(
                arm.moveArmToAngle(12.0)
            ).andThen(
                elevator.goToPosition(30.0)
            ).andThen(
                elevator.goToPosition(45.0)
            ).andThen(
                arm.moveArmToAngle(25.0)
            ).andThen(
                //elevator.goToPosition(10.0).alongWith(
                    arm.moveArmToAngle(170.0)
                //)
            )
        )
        operatorController.leftTrigger().whileTrue(
            arm.moveArmToAngle(90.0).andThen(
                elevator.goToPositionDelta(-10.0)
            )
        )

        // Mark IV controller bindings
        // L1-L4
        markIVController.button(3).onTrue(ReefPositionCommands.l1(elevator, arm))
        markIVController.button(1).onTrue(ReefPositionCommands.l2(elevator, arm))
        markIVController.button(2).onTrue(ReefPositionCommands.l3(elevator, arm))
        markIVController.button(4).onTrue(ReefPositionCommands.l4(elevator, arm))


        // Button macro joystick pad thing
        // L1-4
        buttonMacroController.button(9).onTrue(ReefPositionCommands.l1(elevator, arm))
        buttonMacroController.button(10).onTrue(ReefPositionCommands.l2(elevator, arm))
        buttonMacroController.button(11).onTrue(ReefPositionCommands.l3(elevator, arm))
        buttonMacroController.button(12).onTrue(ReefPositionCommands.l4(elevator, arm))
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
            "FieldSimulation/RobotPosition", driveSimulation!!.simulatedDriveTrainPose
        )
        Logger.recordOutput(
            "FieldSimulation/Coral", *SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
        )
        Logger.recordOutput(
            "FieldSimulation/Algae", *SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
        )
    }
}
