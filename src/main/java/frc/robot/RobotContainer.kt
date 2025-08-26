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
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants.VisionConstants.robotToBackLeftCamera
import frc.robot.Constants.VisionConstants.robotToBackRightCamera
import frc.robot.Constants.VisionConstants.robotToFrontLeftCamera
import frc.robot.Constants.VisionConstants.robotToFrontRightCamera
import frc.robot.automation.AutomationHandler
import frc.robot.automation.SubsystemData
import frc.robot.automation.drivebase.CoralStation
import frc.robot.automation.drivebase.UnifiedReefLocation
import frc.robot.automation.drivebase.requests.BrakeRequest
import frc.robot.automation.drivebase.requests.DriveBackUpRequest
import frc.robot.automation.states.AlgaeStatus
import frc.robot.automation.states.AutoTask
import frc.robot.automation.states.BargePosition
import frc.robot.automation.states.CagePosition
import frc.robot.automation.states.CoralStatus
import frc.robot.automation.states.State
import frc.robot.automation.superstructure.ClawState
import frc.robot.automation.superstructure.Position
import frc.robot.automation.superstructure.SuperstructureState
import frc.robot.automation.superstructure.requests.IntakeState
import frc.robot.automation.superstructure.requests.SuperstructureFixPivotRequest
import frc.robot.automation.superstructure.requests.SuperstructurePickUpCoralRequest
import frc.robot.automation.superstructure.requests.SuperstructureRequest
import frc.robot.automation.superstructure.requests.SuperstructureStateRequest
import frc.robot.commands.DriveCommands
import frc.robot.commands.PathfindingFactories
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.arm.*
import frc.robot.subsystems.climber.*
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorIO
import frc.robot.subsystems.elevator.ElevatorIOSim
import frc.robot.subsystems.elevator.ElevatorIOTalonFX
import frc.robot.subsystems.intake.AlgaeIntake
import frc.robot.subsystems.intake.AlgaeIntakeIO
import frc.robot.subsystems.intake.AlgaeIntakeIOSparkMax
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionIO
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim
import frc.robot.util.*
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.io.print as ioPrint


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@Suppress("GrazieInspection")
class RobotContainer {
    // <editor-fold desc="Subsystems">
    private var drive: Drive
    private var vision: Vision
    private var arm: Arm
    private var elevator: Elevator
    private var algaeIntake: AlgaeIntake
    private var clawIntake: ClawIntake
    private var climber: Climber
    // </editor-fold>


    // <editor-fold desc="Controllers">
    private val driveController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)
    private val hmi = CommandXboxController(2)
    private var hmiEnabled = true
    var enableEmergencyDashboard = false
    // </editor-fold>


    // <editor-fold desc="States">
    private var state: State = State()

    // <editor-fold desc="Misc">
    private val driveQueue: CommandQueue = CommandQueue().withCallback {
        race(
            waitSeconds(0.5), Commands.run({
                driveController.setRumble(
                    GenericHID.RumbleType.kBothRumble, 1.0
                )
            }).finallyDo(
                Runnable { driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) })
        )
    }
    private val superstructureQueue: CommandQueue = CommandQueue().withCallback {
        race(
            waitSeconds(0.5), Commands.run({
                driveController.setRumble(
                    GenericHID.RumbleType.kBothRumble, 1.0
                )
            }).finallyDo(
                Runnable { driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) })
        )
    }.withName("Superstructure Queue")

    private var driveSimulation: SwerveDriveSimulation? = null

    // Dashboard inputs
    private lateinit var autoChooser: LoggedDashboardChooser<Command>
    private var bargeChooser: LoggedDashboardChooser<BargePosition>
    private var cageChooser: LoggedDashboardChooser<CagePosition>

    private val driveTranslationalControlSupplier: Supplier<Translation2d> =
        Supplier<Translation2d> {
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
        @JvmStatic
        val field: Field2d = Field2d()
    }

    // </editor-fold>
    val autoHandler: AutomationHandler

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        println("┌   [RobotContainer] Initializing...")
        val initializeTime: Double = Timer.getFPGATimestamp()
        driveQueue.name = ("Auto Queue")
        var stopTime: Double = Timer.getFPGATimestamp()
        println("╞╦  [RobotContainer] Initializing subsystems at ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")
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
                clawIntake = ClawIntake(
                    ClawIntakeIOSparkMax(Constants.SuperstructureConstants.ClawIntakeConstants.sparkConfig)
                )

                elevator = Elevator(
                    ElevatorIOTalonFX(
                        Constants.SuperstructureConstants.ElevatorConstants.leftMotorConfig,
                        Constants.SuperstructureConstants.ElevatorConstants.rightMotorConfig
                    )
                )

                arm = Arm(
                    ArmIOTalonFX(
                        Constants.SuperstructureConstants.ArmConstants.talonConfig
                    ), elevator.elevatorLigament
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
                clawIntake = ClawIntake(ClawIntakeIOSim())
                elevator = Elevator(ElevatorIOSim())
                climber = Climber(WinchIOSim(), ClimberPivotIOSim())

                arm = Arm(
                    ArmIOSim(

                    ), elevator.elevatorLigament
                )
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
                arm = Arm(object : ArmIO {}, elevator.elevatorLigament)
                clawIntake = ClawIntake(object : ClawIntakeIO {})
                algaeIntake = AlgaeIntake(object : AlgaeIntakeIO {})
                climber = Climber(object : WinchIO {}, object : ClimberPivotIO {})
            }
        }
        println(
            "╞╩ [RobotContainer:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Subsystems initialized in ${
                "%.3f".format(
                    (Timer.getFPGATimestamp() - stopTime) * 1000.0
                )
            }ms"
        )

        autoHandler = AutomationHandler(
            SubsystemData(
                drive,
                arm,
                climber,
                elevator,
                clawIntake,
                driveTranslationalControlSupplier,
                state
            )
        )
        stopTime = Timer.getFPGATimestamp()
        addNamedCommands()
        println(
            "├  [RobotContainer:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Named commands initialized at ${
                "%.3f".format(
                    (Timer.getFPGATimestamp() - stopTime) * 1000.0
                )
            }ms"
        )
        stopTime = Timer.getFPGATimestamp()
        setUpAutoChooser()
        println(
            "├  [RobotContainer:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Auto chooser initialized at ${
                "%.3f".format(
                    (Timer.getFPGATimestamp() - stopTime) * 1000.0
                )
            }ms"
        )
        println("╞╦ [RobotContainer:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Bringing up button bindings!")
        stopTime = Timer.getFPGATimestamp()
        // Configure the button bindings
        configureButtonBindings()
        println(
            "╞╩ [RobotContainer:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Button bindings configured in ${
                "%.3f".format(
                    (Timer.getFPGATimestamp() - stopTime) * 1000.0
                )
            }ms"
        )
        stopTime = Timer.getFPGATimestamp()
        bargeChooser = LoggedDashboardChooser("Barge Position")
        bargeChooser.addOption("Left", BargePosition.LEFT)
        bargeChooser.addOption("Middle", BargePosition.MIDDLE)
        bargeChooser.addOption("Right", BargePosition.RIGHT)
        bargeChooser.addDefaultOption("None", BargePosition.NONE)

        cageChooser = LoggedDashboardChooser("Cage Position")
        cageChooser.addOption("Left", CagePosition.LEFT)
        cageChooser.addOption("Middle", CagePosition.MIDDLE)
        cageChooser.addOption("Right", CagePosition.RIGHT)
        cageChooser.addDefaultOption("None", CagePosition.NONE)
        println(
            "├  [RobotContainer:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Barge/Cage chooser initialized at ${
                "%.3f".format(
                    (Timer.getFPGATimestamp() - stopTime) * 1000.0
                )
            }ms"
        )
        setUpDashboardCommands()

        println("└  [RobotContainer] Initialized in ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")

    }

    // <editor-fold desc="Controller things">
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [CommandXboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        // <editor-fold desc="Drive Controller">
        // Default command, normal field-relative drive
        println("│╠ Setting up default commands")
        drive.defaultCommand = DriveCommands.joystickDrive(
            drive,
            { -driveController.leftY },
            { -driveController.leftX },
            { -driveController.rightX })

        // Default commands for elevator and arm
        elevator.defaultCommand = elevator.stop()
        arm.defaultCommand = arm.stop()
        climber.defaultCommand = climber.moveClimberToIntakePosition()
        clawIntake.defaultCommand = clawIntake.stop()
        ioPrint("│╠ Setting up driver controller bindings... ")
        // Lock to 0° when A button is held
        driveController.a().whileTrue(
            run({ drive.stopWithX() }, drive)
        )
        ioPrint("A (next ones may take a while) ")

        // Switch to X pattern when X button is pressed

        // Pathfinding commands

        // Reduced speed drive when B button is pressed
        driveController.b().onTrue(
            autoHandler.automations.algaeCycle()
        )
        ioPrint("B ")

        driveController.x().onTrue(runOnce({
            if (state.coralStatus == CoralStatus.NONE) state.coralStatus = CoralStatus.ON_INTAKE
        }))
        ioPrint("X ")
        driveController.y().onTrue(
            autoHandler.automations.coralCycle()
        )
        ioPrint("Y ")
        // Reset gyro / odometry
        val resetGyro = if (Constants.currentMode == Constants.Mode.SIM) Runnable {
            drive.pose = driveSimulation!!.simulatedDriveTrainPose
        } else Runnable {
            drive.pose = Pose2d(drive.pose.translation, Rotation2d())
        }
        driveController.start().onTrue(
            runOnce(resetGyro, drive).ignoringDisable(true)
        )
        ioPrint("START ")
        // reset states
        driveController.back().onTrue(runOnce({
            state.coralStatus = CoralStatus.NONE; state.algaeStatus = AlgaeStatus.NONE
        }))
        ioPrint("BACK ")

        // Arm control for left trigger
        driveController.leftTrigger().whileTrue(
            DriveCommands.joystickDrive(
                drive,
                { -driveController.leftY * 0.25 },
                { -driveController.leftX * 0.25 },
                { -driveController.rightX * 0.25 })
        )
        ioPrint("LT ")
//        driveController.pov(90).onTrue(driveQueue.addButDoNotStartAsCommand({
//            PathfindingFactories.pathfindToCoralStation(
//                drive,
//                PathfindingFactories.CoralStationSide.RIGHT,
//                driveTranslationalControlSupplier
//            )
//        }))
//        driveController.pov(180).onTrue(driveQueue.addButDoNotStartAsCommand({
//            PathfindingFactories.pathfindToCoralStation(
//                drive, PathfindingFactories.CoralStationSide.LEFT, driveTranslationalControlSupplier
//            )
//        }))
        driveController.rightBumper()
            .onTrue(runOnce({ state.nextStation = CoralStation.RIGHT }))
        ioPrint("RB ")
        driveController.leftBumper()
            .onTrue(runOnce({ state.nextStation = CoralStation.LEFT }))
        ioPrint("LB ")
        driveController.povDown().onTrue(
            runOnce({
                state.nextCage = cageChooser.get()
                state.push(task = AutoTask.TO_CAGE)
            }).andThen(
                defer({
                    PathfindingFactories.pathfindToPosition(
                        drive,
                        Constants.PathfindingConstants.getOtherPosition(state.nextCage),
                        driveTranslationalControlSupplier
                    ).deadlineFor(
                        autoHandler.accept(SuperstructureStateRequest(Position.ALGAE_INTAKE))
                    )
                }, setOf(drive, elevator, arm, climber))
            ).finallyDo(Runnable { state.push(task = AutoTask.IDLE) })
        )
        ioPrint("POV_DOWN ")
        driveController.povUp().onTrue(autoHandler.automations.scoreCoral())
        println("POV_UP - done")
        // </editor-fold>


        //Trigger { abs(operatorController.leftY) > 0.1 }.whileTrue(
        //    arm.moveArm { -operatorController.leftY }
        //)
        //Trigger { abs(operatorController.rightY) > 0.1 }.whileTrue(
        //    elevator.velocityCommand { -operatorController.rightY }
        //)

        ioPrint("│╠ Setting up operator controller bindings... ")
        // Operator controller bindings
        operatorController.a().whileTrue(algaeIntake.runIntake())
        ioPrint("A ")
        operatorController.b().onTrue(algaeIntake.retractIntake())
        ioPrint("B ")
        operatorController.x().onTrue(
            climber.climb()
        )
        ioPrint("X ")
        operatorController.leftTrigger().whileTrue(
            climber.moveClimberToCageCatchPositionNoStop()
        )
        ioPrint("LT ")

        operatorController.rightTrigger().whileTrue(
            elevator.velocityCommand { -operatorController.rightY }.alongWith(
                arm.moveArm { -operatorController.leftY },
            ),
        )
        ioPrint("RT ")
        operatorController.leftBumper()
            .whileTrue(clawIntake.spinFlywheel(Constants.SuperstructureConstants.ClawIntakeConstants.clawIntakePercent * 1.5))
        ioPrint("LB ")
        operatorController.rightBumper()
            .whileTrue(clawIntake.spinFlywheel(-Constants.SuperstructureConstants.ClawIntakeConstants.clawIntakePercent * 2.0))
        ioPrint("RB ")

        operatorController.start().whileTrue(climber.moveClimberOpenLoop({ -0.5 }, { 0.0 }))
        ioPrint("START ")
        operatorController.back().whileTrue(climber.moveClimberOpenLoop({ 0.5 }, { 0.0 }))
        println("BACK - done")

        // Mark IV controller bindings


        ioPrint("│╠ Setting up HMI bindings... ")
        hmi.b().onTrue(runOnce({
            state.nextPosition = Position.L2
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.y().onTrue(runOnce({
            state.nextPosition = Position.L3
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.x().onTrue(runOnce({
            state.nextPosition = Position.L4
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        // deprecated
//        hmi.a().onTrue(runOnce({ state.keepGoing = true }).onlyIf(::hmiEnabled))
//            .onFalse(runOnce({ keepGoing = false }).onlyIf(::hmiEnabled))
        ioPrint("levels, ")

        hmi.povUp().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.povUpRight().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.povDownRight().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.povDown().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.povDownLeft().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.povUpLeft().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.leftBumper().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.rightBumper().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        println("positions - done")

        //Trigger { drive.velocity > 2.0 && elevator.currentCommand == elevator.defaultCommand }.onTrue(
        //Trigger { drive.velocity > 2.0 && elevator.currentCommand == elevator.defaultCommand }.onTrue(
        //    SuperstructureCommands.home(elevator, arm)
        //)


    }


    fun updateHmiAlgae() {
        // [0, 0.05]
        // between 0 and 6
        if (hmiEnabled) {
            val joystickValue = (hmi.getRawAxis(0) / 0.05 * 6).roundToInt()
            if (joystickValue == 0) {
                state.nextAlgaePosition = UnifiedReefLocation.NONE
            } else {
                state.nextAlgaePosition = when (joystickValue) {
                    1 -> UnifiedReefLocation.AB
                    2 -> UnifiedReefLocation.CD
                    3 -> UnifiedReefLocation.EF
                    4 -> UnifiedReefLocation.GH
                    5 -> UnifiedReefLocation.IJ
                    else -> UnifiedReefLocation.KL
                }
            }
        }
        state.updateIO()
    }

    private fun updateHmi() {
        if (hmi.povUp().asBoolean) {
            state.nextReef = when (hmi.leftBumper().asBoolean) {
                true -> UnifiedReefLocation.G
                false -> UnifiedReefLocation.H
            }
        }
        if (hmi.povUpRight().asBoolean) {
            state.nextReef = when (hmi.leftBumper().asBoolean) {
                true -> UnifiedReefLocation.E
                false -> UnifiedReefLocation.F
            }
        }
        if (hmi.povDownRight().asBoolean) {
            state.nextReef = when (hmi.leftBumper().asBoolean) {
                true -> UnifiedReefLocation.C
                false -> UnifiedReefLocation.D
            }
        }
        if (hmi.povDown().asBoolean) {
            state.nextReef = when (hmi.leftBumper().asBoolean) {
                true -> UnifiedReefLocation.A
                false -> UnifiedReefLocation.B
            }
        }
        if (hmi.povDownLeft().asBoolean) {
            state.nextReef = when (hmi.leftBumper().asBoolean) {
                true -> UnifiedReefLocation.K
                false -> UnifiedReefLocation.L
            }
        }
        if (hmi.povUpLeft().asBoolean) {
            state.nextReef = when (hmi.leftBumper().asBoolean) {
                true -> UnifiedReefLocation.I
                false -> UnifiedReefLocation.J
            }
        }
    }
    // </editor-fold>


    // <editor-fold desc="Auto Registration">
    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()


    private fun createPathfindingAuto(
        bargeAdjective: String,
        stationSide: CoralStation,
        initialReef: UnifiedReefLocation,
        vararg reefs: UnifiedReefLocation
    ): Command {
        var auto: Command = AutoBuilder.buildAuto(
            "Start at $bargeAdjective ${
                when (stationSide) {
                    CoralStation.LEFT -> "Left"
                    CoralStation.RIGHT -> "Right"
                }
            } and Score Coral ${
                when (stationSide) {
                    CoralStation.LEFT -> "Left"
                    CoralStation.RIGHT -> "Right"
                }
            } But End Halfway Through"
        ).withTimeout(5.0).andThen({
            updateReef(initialReef)
            state.nextPosition = Position.L4
            state.nextStation = stationSide
        })

        for (reef in reefs.copyOfRange(0, reefs.size - 1)) {
            auto = auto.andThen(
                autoHandler.automations.coralCycle(true).alongWith(
                    waitSeconds(0.75).andThen(
                        { updateReef(reef) })
                )
            )
        }

        auto = auto.andThen({ state.nextReef = reefs.last() })
            .andThen(autoHandler.automations.coralCycle(true))
        return auto
    }

    private fun setUpAutoChooser() {
        // Set up auto routines
        autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())

        // Set up SysId routines
        // <editor-fold desc="SysId Routines">

        autoChooser.addOption(
            "Pathfinding Auto (Far Left, Left, All)", createPathfindingAuto(
                "Far",
                CoralStation.LEFT,
                UnifiedReefLocation.J,
                UnifiedReefLocation.K,
                UnifiedReefLocation.L,
                UnifiedReefLocation.A
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Far Left, Left, Home Field)", createPathfindingAuto(
                "Far",
                CoralStation.LEFT,
                UnifiedReefLocation.J,
                UnifiedReefLocation.A,
                UnifiedReefLocation.B
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Left, All)", createPathfindingAuto(
                "Mid",
                CoralStation.LEFT,
                UnifiedReefLocation.J,
                UnifiedReefLocation.K,
                UnifiedReefLocation.L,
                UnifiedReefLocation.A
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Left, Home Field)", createPathfindingAuto(
                "Mid",
                CoralStation.LEFT,
                UnifiedReefLocation.J,
                UnifiedReefLocation.A,
                UnifiedReefLocation.B
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Algae, All)", createPathfindingAuto(
                "Mid",
                CoralStation.LEFT,
                UnifiedReefLocation.J,
                UnifiedReefLocation.K
            ).andThen(
                { state.push(nextAlgaePosition = UnifiedReefLocation.KL) }
            ).andThen(
                autoHandler.automations.algaeCycle()
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Algae, Home Field)", createPathfindingAuto(
                "Mid",
                CoralStation.LEFT,
                UnifiedReefLocation.J,
                UnifiedReefLocation.A,
            ).andThen(
                { state.push(nextAlgaePosition = UnifiedReefLocation.AB) }
            ).andThen(
                autoHandler.automations.algaeCycle()
            )
        )
        autoChooser.addOption(
            "Pathfinding Auto (Far Right, Right, All)", createPathfindingAuto(
                "Far",
                CoralStation.RIGHT,
                UnifiedReefLocation.E,
                UnifiedReefLocation.D,
                UnifiedReefLocation.C,
                UnifiedReefLocation.B,
                UnifiedReefLocation.A
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Right, Right, All)", createPathfindingAuto(
                "Mid",
                CoralStation.RIGHT,
                UnifiedReefLocation.E,
                UnifiedReefLocation.D,
                UnifiedReefLocation.C,
                UnifiedReefLocation.B,
                UnifiedReefLocation.A
            )
        )
        autoChooser.addOption(
            "Pathfinding Auto (Mid, Algae)",
            sequence(
                runOnce({
                    state.nextReef = UnifiedReefLocation.H
                    state.coralStatus = CoralStatus.IN_CLAW
                    state.nextAlgaePosition = UnifiedReefLocation.GH
                }),
                autoHandler.automations.coralCycle(false),
                autoHandler.automations.algaeCycle()
            )
        )

        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive)
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive)
        )
        // </editor-fold>

    }

    fun setUpDashboardCommands() {
        SmartDashboard.putData(
            "Log Drive Time",
            runOnce({ drive.logCycleTime = true }).ignoringDisable(true)
        )
        SmartDashboard.putData(
            elevator.homeElevator().deadlineFor(arm.moveArmToAngleWithoutEnding(90.0))
                .withName("Home Elevator")
        )
        SmartDashboard.putData(CommandScheduler.getInstance())

        SmartDashboard.putData(
            runOnce({
                enableEmergencyDashboard = true
                Elastic.selectTab("Emergency Dashboard")
            }).ignoringDisable(true).withName("Enable Emergency Dashboard")
        )
    }

    fun emergencyDashboardSetup() {
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.A }).withName("\nSelect Reef A\n")
        )
        // repeat for other reefs
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.B }).withName("\nSelect Reef B\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.C }).withName("\nSelect Reef C\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.D }).withName("\nSelect Reef D\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.E }).withName("\nSelect Reef E\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.F }).withName("\nSelect Reef F\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.G }).withName("\nSelect Reef G\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.H }).withName("\nSelect Reef H\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.I }).withName("\nSelect Reef I\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.J }).withName("\nSelect Reef J\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.K }).withName("\nSelect Reef K\n")
        )
        SmartDashboard.putData(
            runOnce({ state.nextReef = UnifiedReefLocation.L }).withName("\nSelect Reef L\n")
        )

        // algae as well
        SmartDashboard.putData(
            runOnce({
                state.nextAlgaePosition = UnifiedReefLocation.AB;
            }).withName("\nSelect Algae AB\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextAlgaePosition = UnifiedReefLocation.CD;
            }).withName("\nSelect Algae CD\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextAlgaePosition = UnifiedReefLocation.EF;
            }).withName("\nSelect Algae EF\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextAlgaePosition = UnifiedReefLocation.GH;
            }).withName("\nSelect Algae GH\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextAlgaePosition = UnifiedReefLocation.IJ;
            }).withName("\nSelect Algae IJ\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextAlgaePosition = UnifiedReefLocation.KL;
            }).withName("\nSelect Algae KL\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextAlgaePosition = UnifiedReefLocation.NONE
            }).withName("\nDisable Algae\n")
        )

        // finally, positions
        SmartDashboard.putData(
            runOnce({
                state.nextPosition = Position.L1
            }).withName("\nSelect Position L1\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextPosition = Position.L2
            }).withName("\nSelect Position L2\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextPosition = Position.L3
            }).withName("\nSelect Position L3\n")
        )
        SmartDashboard.putData(
            runOnce({
                state.nextPosition = Position.L4
            }).withName("\nSelect Position L4\n")
        )
    }


    private fun addNamedCommands() {
        NamedCommands.registerCommand(
            "home", autoHandler.accept(SuperstructureStateRequest(Position.HOME))
        )
        NamedCommands.registerCommand(
            "L4", autoHandler.accept(SuperstructureStateRequest(Position.L4))
        )
        NamedCommands.registerCommand(
            "deposit", autoHandler.accept(
                SuperstructureRequest(SuperstructureStates.L4_SCORE),
                DriveBackUpRequest()
            )
        )
        NamedCommands.registerCommand(
            "Pre-Coral Pickup",
            autoHandler.accept(SuperstructureStateRequest(Position.PRE_CORAL_PICKUP))
        )
        NamedCommands.registerCommand(
            "Fix Pivot",
            autoHandler.accept(SuperstructureFixPivotRequest())
        )
        NamedCommands.registerCommand(
            "Fix Pivot and L4", autoHandler.accept(SuperstructureFixPivotRequest(scoreOnL4 = true))
        )
        NamedCommands.registerCommand(
            "Pick Up and L4",
            autoHandler.accept(SuperstructurePickUpCoralRequest()).withTimeout(3.75).andThen(
                autoHandler.accept(SuperstructureStateRequest(Position.L4))
            )
        )
        NamedCommands.registerCommand(
            "Low Algae",
            autoHandler.accept(SuperstructureStateRequest(Position.LOWER_REEF_ALGAE))
                .alongWith(
                    autoHandler.accept(SuperstructureRequest(intakeSpeed = IntakeState.ALGAE_INTAKE))
                )
        )
        NamedCommands.registerCommand(
            "Spit Out",
            autoHandler.accept(SuperstructureRequest(intakeSpeed = IntakeState.OUTTAKE))
        )
        NamedCommands.registerCommand(
            "Barge Algae",
            autoHandler.accept(SuperstructureStateRequest(Position.BARGE_LAUNCH))
        )
        NamedCommands.registerCommand(
            "Lock Wheels", autoHandler.accept(BrakeRequest())
        )
        NamedCommands.registerCommand(
            "Run Intake", autoHandler.accept(
                SuperstructureRequest(intakeSpeed = IntakeState.INTAKE)
            )
        )
    }
    // </editor-fold>

    // <editor-fold desc="Miscellaneous Utilities">
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

    fun stopQueue() {
        driveQueue.clearAll()
        superstructureQueue.clearAll()
    }

    fun fixArm() {
        if (climber.position > 90.0) autoHandler.accept(SuperstructureFixPivotRequest()).schedule()
    }

    fun disableAuto() {
        state.coralStatus = CoralStatus.NONE
    }
    // </editor-fold>

    // <editor-fold desc="Auto functions">

    // <editor-fold desc="Full Cycles">

    private fun updateReef(reef: UnifiedReefLocation) {
        state.nextReef = reef
    }
    // </editor-fold>
    // </editor-fold>
}


// Algae format for HMI input goes AB reef first, counterclockwise
