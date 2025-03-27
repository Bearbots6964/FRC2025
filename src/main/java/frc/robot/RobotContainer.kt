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
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants.VisionConstants.robotToBackLeftCamera
import frc.robot.Constants.VisionConstants.robotToBackRightCamera
import frc.robot.Constants.VisionConstants.robotToFrontLeftCamera
import frc.robot.Constants.VisionConstants.robotToFrontRightCamera
import frc.robot.commands.*
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.arm.*
import frc.robot.subsystems.climber.*
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorIO
import frc.robot.subsystems.elevator.ElevatorIOTalonFX
import frc.robot.subsystems.intake.AlgaeIntake
import frc.robot.subsystems.intake.AlgaeIntakeIO
import frc.robot.subsystems.intake.AlgaeIntakeIOSparkMax
import frc.robot.subsystems.vision.*
import frc.robot.util.CommandQueue
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier
import kotlin.math.hypot
import kotlin.math.pow


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
    private var algaeIntake: AlgaeIntake
    private var clawIntake: ClawIntake
    private var climber: Climber

    private var driveSimulation: SwerveDriveSimulation? = null

    // Controller
    private val driveController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)

    private val autoQueue: CommandQueue = CommandQueue().withCallback(Commands.race(
        Commands.waitSeconds(0.5),
        Commands.run(Runnable { driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) }).finallyDo(
            Runnable{ driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)})
    ))
    private val otherCommandQueue: CommandQueue = CommandQueue().withCallback(Commands.race(
        Commands.waitSeconds(0.5),
        Commands.run(Runnable { driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) }).finallyDo(
            Runnable{ driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)})
    )).withName("Other Command Queue")

    // Dashboard inputs
    private lateinit var autoChooser: LoggedDashboardChooser<Command>

    private var nextSuperstructureCommand: Constants.SuperstructureConstants.SuperstructureState =
        Constants.SuperstructureConstants.SuperstructureState.HOME

    private val speedAt12Volts = TunerConstants.speedAt12Volts.`in`(Units.MetersPerSecond)

    val driveTranslationalControlSupplier: Supplier<Translation2d> = Supplier<Translation2d> {
        var xControl: Double = -driveController.leftY
        var yControl: Double = -driveController.leftX
        val magnitude = hypot(xControl, yControl)
        if (magnitude > 1) {
            xControl /= magnitude
            yControl /= magnitude
        } else if (magnitude > 1e-6) {
            val scalar = (MathUtil.applyDeadband(magnitude, .06) / magnitude).pow(2.0)
            xControl *= scalar
            yControl *= scalar
        }
        Translation2d(xControl, yControl)
    }

    companion object {


    }


    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        autoQueue.setName("Auto Queue")
        SmartDashboard.putData(CommandScheduler.getInstance())
        SmartDashboard.putData("Auto Queue", autoQueue)
        SmartDashboard.putData("Other Command Queue", otherCommandQueue)
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
                    drive, VisionIOPhotonVision(
                        Constants.VisionConstants.frontRightCameraName, robotToFrontRightCamera
                    ), VisionIOPhotonVision(
                        Constants.VisionConstants.backRightCameraName, robotToBackRightCamera
                    ), VisionIOPhotonVision(
                        Constants.VisionConstants.frontLeftCameraName, robotToFrontLeftCamera
                    ), VisionIOPhotonVision(
                        Constants.VisionConstants.backLeftCameraName, robotToBackLeftCamera
                    )
                )
                arm = Arm(
                    ArmIOTalonFX(
                        Constants.SuperstructureConstants.ArmConstants.talonConfig
                    )
                )
                clawIntake = ClawIntake(
                    ClawIntakeIOSparkMax(Constants.SuperstructureConstants.ClawIntakeConstants.sparkConfig)
                )

                elevator = Elevator(
                    ElevatorIOTalonFX(
                        Constants.SuperstructureConstants.ElevatorConstants.leftMotorConfig,
                        Constants.SuperstructureConstants.ElevatorConstants.rightMotorConfig
                    )
                )

                algaeIntake = AlgaeIntake(
                    AlgaeIntakeIOSparkMax(
                        Constants.AlgaeIntakeConstants.armConfig,
                        Constants.AlgaeIntakeConstants.intakeConfig
                    )
                )
                climber = Climber(
                    WinchIOTalonFX(Constants.ClimberConstants.winchMotorConfig),
                    ClimberPivotIOTalonFX(Constants.ClimberConstants.pivotMotorConfig)
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
                        Constants.VisionConstants.frontRightCameraName, robotToFrontRightCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose }, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.backRightCameraName, robotToBackRightCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose }, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.frontLeftCameraName, robotToFrontLeftCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose }, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.backLeftCameraName, robotToBackLeftCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose })
                arm = Arm(
                    ArmIOTalonFXSim(

                    )
                )
                clawIntake = ClawIntake(object : ClawIntakeIO {})
                elevator = Elevator(object : ElevatorIO {})
                climber = Climber(object : WinchIO {}, object : ClimberPivotIO {})

                algaeIntake = AlgaeIntake(object : AlgaeIntakeIO {})
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
                clawIntake = ClawIntake(object : ClawIntakeIO {})
                algaeIntake = AlgaeIntake(object : AlgaeIntakeIO {})
                climber = Climber(object : WinchIO {}, object : ClimberPivotIO {})
            }
        }
        addNamedCommands()

        setUpAutoChooser()


        // Configure the button bindings
        configureButtonBindings()

        setUpDashboardCommands()

    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        // <editor-fold desc="Drive Controller">
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
        driveController.x().onTrue(Commands.runOnce({ autoQueue.start() }))

        // Pathfinding commands

        // Reduced speed drive when B button is pressed
        driveController.b().onTrue(
            DriveCommands.joystickDrive(
                drive,
                { -driveController.leftY * 0.5 },
                { -driveController.leftX * 0.5 },
                { -driveController.rightX })
        )


        // Arm control after coral placed
        driveController.rightTrigger().onTrue(
            (if (SuperstructureCommands.reefPosition != Constants.SuperstructureConstants.SuperstructureState.L4) elevator.goToPositionDelta(
                -10.0
            )
            else elevator.goToPositionDelta(10.0)).withName("Move Elevator Down")
                .alongWith(arm.moveArmAngleDelta(-30.0).withName("Move Arm Down"))
                .alongWith(drive.backUpBy().withName("Back Up"))
        )

        // Arm control for left trigger
        driveController.leftTrigger().whileTrue(
            DriveCommands.joystickDrive(
                drive,
                { -driveController.leftY * 0.25 },
                { -driveController.leftX * 0.25 },
                { -driveController.rightX * 0.25 })
        )

        // Reset gyro / odometry
        val resetGyro = if (Constants.currentMode == Constants.Mode.SIM) Runnable {
            drive.pose = driveSimulation!!.simulatedDriveTrainPose
        } else Runnable {
            drive.pose = Pose2d(drive.pose.translation, Rotation2d())
        }
        driveController.start().onTrue(
            Commands.runOnce(resetGyro, drive).ignoringDisable(true)
        )

        driveController.pov(90).onTrue(autoQueue.addButDoNotStartAsCommand({ PathfindingFactories.pathfindToCoralStationAlternate(drive, PathfindingFactories.CoralStationSide.RIGHT, driveTranslationalControlSupplier)}))
        driveController.pov(180).onTrue(autoQueue.addButDoNotStartAsCommand({ PathfindingFactories.pathfindToCoralStationAlternate(drive, PathfindingFactories.CoralStationSide.LEFT, driveTranslationalControlSupplier)}))
        // </editor-fold>

        // Default commands for elevator and arm
        elevator.defaultCommand = elevator.stop()
        arm.defaultCommand = arm.stop()
        climber.defaultCommand = climber.moveClimberToIntakePosition()
        clawIntake.defaultCommand = clawIntake.stop()

        //Trigger { abs(operatorController.leftY) > 0.1 }.whileTrue(
        //    arm.moveArm { -operatorController.leftY }
        //)
        //Trigger { abs(operatorController.rightY) > 0.1 }.whileTrue(
        //    elevator.velocityCommand { -operatorController.rightY }
        //)

        // Operator controller bindings
        operatorController.a().onTrue(algaeIntake.runIntake())
        operatorController.b().onTrue(algaeIntake.retractIntake())
        operatorController.a().whileTrue(
            SuperstructureCommands.home(elevator, arm, climber)
        )
        operatorController.b().whileTrue(Commands.runOnce({
            otherCommandQueue.cancelCurrent()
        }))
        driveController.y().onTrue(Commands.runOnce({
            otherCommandQueue.start()
        }))
        operatorController.y().whileTrue(Commands.runOnce({
            nextSuperstructureCommand = Constants.SuperstructureConstants.SuperstructureState.L4
        }))
        operatorController.rightTrigger().whileTrue(
            elevator.velocityCommand({ -operatorController.rightY }).alongWith(
                arm.moveArm({ -operatorController.leftY }),
            ),
        )
        operatorController.leftTrigger().onTrue(
            Commands.runOnce({ clawIntake.setIntakeGrabbed(true) }),
        ).onFalse(
            Commands.runOnce({ clawIntake.setIntakeGrabbed(false) }),
        )

        operatorController.leftBumper()
            .whileTrue(clawIntake.spinFlywheel(Constants.SuperstructureConstants.ClawIntakeConstants.clawIntakePercent * 1.5))
        operatorController.rightBumper()
            .whileTrue(clawIntake.spinFlywheel(-Constants.SuperstructureConstants.ClawIntakeConstants.clawIntakePercent * 2.0))

        operatorController.leftStick().whileTrue(climber.moveClimberToCageCatchPositionNoStop())
        operatorController.rightStick().onTrue(climber.climb())
        operatorController.start().whileTrue(climber.moveClimberOpenLoop({ -0.5 }, { 0.0 }))

        // Mark IV controller bindings


        //Trigger { drive.velocity > 2.0 && elevator.currentCommand == elevator.defaultCommand }.onTrue(
        //    SuperstructureCommands.home(elevator, arm)
        //)
    }

    private fun coralPickup(): SequentialCommandGroup = elevator.goToPosition(60.0).andThen(
        arm.moveArmToAngle(8.0)
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

    private fun setUpAutoChooser() {
        // Set up auto routines
        autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())

        // Set up SysId routines
        // <editor-fold desc="SysId Routines">
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

        autoChooser.addOption(
            let {
                var a = ""
                for (i in 0..1000) a += "1234567890"
                a
            }, Commands.none()
        )
    }

    private fun setUpDashboardCommands() {




        SmartDashboard.putData(
            elevator.homeElevator()
                .deadlineFor(arm.moveArmToAngleWithoutEnding(90.0)).withName("Home Elevator")
        )

        SmartDashboard.putData(drive.followRepulsorField(AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[2]))


        SmartDashboard.putData(algaeIntake.runIntake().withName("Run Intake"))
        SmartDashboard.putData(algaeIntake.retractIntake().withName("Retract Intake"))


        // add individual reef goto commands to queue
        //for (reef in PathfindingFactories.Reef.entries) {
        //    SmartDashboard.putData(
        //        autoQueue.addAsCommand({
        //            PathfindingFactories.pathfindToSpecificReef(drive, reef)
        //                .withName("Reef " + reef.name + " (Queued)")
        //        }).withName("Queue Reef " + reef.name)
        //    )
        //}
        for (reef in PathfindingFactories.Reef.entries) {
            SmartDashboard.putData(
                autoQueue.addButDoNotStartAsCommand({
                    PathfindingFactories.pathfindToReefAlternate(
                        drive, reef, driveTranslationalControlSupplier
                    ).withName("Reef " + reef.name + " (Queued, alternate)")
                }).withName("Queue Reef alternate " + reef.name).ignoringDisable(true)
            )
        }

        //for (reef in PathfindingFactories.Reef.entries) {
        //    SmartDashboard.putData(
        //        autoQueue.addAsCommand({
        //            PathfindingFactories.finalLineupToSpecificReef(drive, reef)
        //                .withName("Final Lineup to Reef " + reef.name + " (Queued)")
        //        }).withName("Queue Final Lineup to Reef " + reef.name)
        //    )
        //}
        for (coralStation in PathfindingFactories.CoralStationSide.entries) {
//            SmartDashboard.putData(
//                autoQueue.addAsCommand({
//                    PathfindingFactories.pathfindToSpecificCoralStation(
//                        drive, coralStation
//                    ).withName("Drive to " + coralStation.name + " Coral Station (Queued)")
//                }).withName("Queue Drive to " + coralStation.name + " Coral Station")
//            )
            SmartDashboard.putData(
                autoQueue.addButDoNotStartAsCommand({
                    PathfindingFactories.pathfindToCoralStationAlternate(
                        drive, coralStation, driveTranslationalControlSupplier
                    ).withName("Drive to $coralStation Coral Station (Queued, alternate)")
                }).ignoringDisable(true)
                    .alongWith(
                        otherCommandQueue.addButDoNotStartAsCommand({
                            SuperstructureCommands.algaeIntake(elevator, arm, climber).withName("Superstructure Algae Intake (queued, auto-added)")
                        }).withName("Queue Superstructure Algae Intake").ignoringDisable(true)
                    ).andThen(

                        otherCommandQueue.addButDoNotStartAsCommand({
                            SuperstructureCommands.pickUpCoral(elevator, arm, clawIntake, climber).withName("Superstructure Coral Pickup (queued, auto-added)")
                        }).withName("Queue Superstructure Algae Intake").ignoringDisable(true)
                    ).withName("Queue Drive to $coralStation Coral Station (alternate)")
            )
        }/*
        for (coralStation in PathfindingFactories.CoralStationSide.entries) {
            SmartDashboard.putData(
                autoQueue.addAsCommand({
                    PathfindingFactories.finalLineupToSpecificCoralStation(
                        drive, coralStation
                    ).withName("Final Lineup to " + coralStation.name + " Coral Station (Queued)")
                }).withName("Queue Final Lineup to " + coralStation.name + " Coral Station")
            )
        }
        */


        for (position in Constants.SuperstructureConstants.SuperstructureState.entries) {
            val commands: Command =
                when (position) {
                    Constants.SuperstructureConstants.SuperstructureState.L1, Constants.SuperstructureConstants.SuperstructureState.L2, Constants.SuperstructureConstants.SuperstructureState.L3, Constants.SuperstructureConstants.SuperstructureState.L4 -> otherCommandQueue.addButDoNotStartAsCommand(
                        {
                            SuperstructureCommands.goToPosition(
                                elevator, arm, climber, position
                            ).withName("Superstructure to " + position.name + " Position (Queued)")
                        },
                        {
                            SuperstructureCommands.scoreAtPosition(
                                elevator, arm, clawIntake, drive, position
                            ).withName("Score in $position (queued, auto-added)")
                        })
                    Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE, Constants.SuperstructureConstants.SuperstructureState.UPPER_REEF_ALGAE -> otherCommandQueue.addButDoNotStartAsCommand(
                        {
                            SuperstructureCommands.goToPosition(elevator, arm, climber, position).withName("Superstructure to $position Position (Queued)")
                        },
                        {
                            clawIntake.intakeWithoutStoppingForAlgae().withName("Intake Algae (Queued, auto-added")
                        },
                        {
                            SuperstructureCommands.goToPosition(elevator, arm, climber, Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH).withName("Superstructure to Barge Launch Position (Queued, auto-added)")
                        }
                    )
                    else -> otherCommandQueue.addButDoNotStartAsCommand({
                        SuperstructureCommands.goToPosition(
                            elevator, arm, climber, position
                        ).withName("Superstructure to " + position.name + " Position (Queued)")
                    })
                }
            SmartDashboard.putData(
                commands.withName("Queue Superstructure $position Position").ignoringDisable(true)
            )
        }
        SmartDashboard.putData(otherCommandQueue.addButDoNotStartAsCommand({
            SuperstructureCommands.pickUpCoral(
                elevator, arm, clawIntake, climber
            ).withName("Pick up coral (queued)")
        }).withName("Queue pick up coral").ignoringDisable(true))
        SmartDashboard.putData(otherCommandQueue.addButDoNotStartAsCommand({
            SuperstructureCommands.score(
                elevator, arm, clawIntake
            ).withName("Score (queued)")
        }).withName("Queue score").ignoringDisable(true))

        if (Constants.currentMode == Constants.Mode.SIM) SmartDashboard.putData(Commands.runOnce({autoQueue.start()}).withName("Execute (sim-exclusive)"))

    }


    fun addNamedCommands() {
        NamedCommands.registerCommand("home", SuperstructureCommands.home(elevator, arm, climber))
        NamedCommands.registerCommand("L4", SuperstructureCommands.l4WithoutSafety(elevator, arm))
        NamedCommands.registerCommand("deposit", SuperstructureCommands.scoreAtPositionWithoutDrive(elevator, arm, clawIntake, Constants.SuperstructureConstants.SuperstructureState.L4))
        NamedCommands.registerCommand(
            "Pre-Coral Pickup", SuperstructureCommands.preCoralPickup(elevator, arm, climber)
        )
        NamedCommands.registerCommand("Fix Pivot", elevator.goToPosition(40.0).alongWith(climber.pivotToPosition(57.0)).andThen(SuperstructureCommands.algaeIntakeWithoutSafety(elevator, arm, climber)))
        NamedCommands.registerCommand("Fix Pivot and L4", climber.pivotToPosition(57.0).alongWith(SuperstructureCommands.l4WithoutSafety(
            elevator,
            arm
        )))
        NamedCommands.registerCommand("Pick Up and L4", SuperstructureCommands.pickUpCoral(elevator, arm, clawIntake, climber).andThen(SuperstructureCommands.l4WithoutSafety(
            elevator,
            arm
        )))
    }

    fun stopQueue() {
        autoQueue.clearAll()
        otherCommandQueue.clearAll()
    }
}
