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
import frc.robot.commands.DriveCommands
import frc.robot.commands.PathfindingFactories
import frc.robot.commands.SuperstructureCommands
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
    private var nextReef: PathfindingFactories.Reef
        get() = state.nextReef
        set(value) {
            state.push(nextReef = value)
        }
    private var nextStation: PathfindingFactories.CoralStationSide
        get() = state.nextStation
        set(value) {
            state.push(nextStation = value)
        }
    private var nextPosition: Constants.SuperstructureConstants.SuperstructureState =
        Constants.SuperstructureConstants.SuperstructureState.PRE_CORAL_PICKUP
        set(value) {
            field = value
            state.push(nextState = value)
        }
    private var nextAlgaePosition: PathfindingFactories.Reef
        get() = state.nextAlgaePosition
        set(value) {
            state.push(nextAlgaePosition = value)
        }
    private var grabAlgaeToggle = false
    private var keepGoing = false

    private var coralStatus: CoralStatus
        get() = state.coralStatus
        set(value) {
            state.push(coralStatus = value)
        }
    private var algaeStatus: AlgaeStatus
        get() = state.algaeStatus
        set(value) {
            state.push(algaeStatus = value)
        }
    private var bargePosition: BargePosition
        get() = state.nextBarge
        set(value) {
            state.push(nextBarge = value)
        }
    private var nextCage: CagePosition
        get() = state.nextCage
        set(value) {
            state.push(nextCage = value)
        }
    // </editor-fold>

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

    private val driveTranslationalControlSupplier: Supplier<Translation2d> = Supplier<Translation2d> {
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

    private val superstructureCommands: SuperstructureCommands =
        SuperstructureCommands { state.push(currentState = it) }

    companion object {
        @JvmStatic
        val field: Field2d = Field2d()
    }
    // </editor-fold>

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
                        Constants.AlgaeIntakeConstants.armConfig, Constants.AlgaeIntakeConstants.intakeConfig
                    )
                )
                climber = Climber(
                    WinchIOTalonFX(Constants.ClimberConstants.winchMotorConfig),
                    ClimberPivotIOTalonFX(Constants.ClimberConstants.pivotMotorConfig)
                )
            }

            Constants.Mode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = SwerveDriveSimulation(Drive.mapleSimConfig, Pose2d(3.0, 3.0, Rotation2d()))
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

    private fun posToCageCommand(pos: CagePosition): Command {
        return PathfindingFactories.pathfindToPosition(
            drive, Constants.PathfindingConstants.getOtherPosition(pos), driveTranslationalControlSupplier
        ).deadlineFor(
            superstructureCommands.goToPosition(
                elevator, arm, climber, Constants.SuperstructureConstants.SuperstructureState.ALGAE_INTAKE
            )
        )
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
            algaeCycle()
        )
        ioPrint("B ")

        driveController.x().onTrue(runOnce({
            if (coralStatus == CoralStatus.NONE) coralStatus = CoralStatus.ON_INTAKE
        }))
        ioPrint("X ")
        driveController.y().onTrue(
            runOneFullCoralCycle()
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
        driveController.back().onTrue(runOnce({ coralStatus = CoralStatus.NONE; algaeStatus = AlgaeStatus.NONE }))
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
        driveController.rightBumper().onTrue(runOnce({ nextStation = PathfindingFactories.CoralStationSide.RIGHT }))
        ioPrint("RB ")
        driveController.leftBumper().onTrue(runOnce({ nextStation = PathfindingFactories.CoralStationSide.LEFT }))
        ioPrint("LB ")
        driveController.povDown().onTrue(
            runOnce({
                nextCage = cageChooser.get()
                state.push(task = AutoTask.TO_CAGE)
            }).andThen(
                defer({
                    PathfindingFactories.pathfindToPosition(
                        drive,
                        Constants.PathfindingConstants.getOtherPosition(nextCage),
                        driveTranslationalControlSupplier
                    ).deadlineFor(
                        superstructureCommands.goToPosition(
                            elevator, arm, climber, Constants.SuperstructureConstants.SuperstructureState.ALGAE_INTAKE
                        )
                    )
                }, setOf(drive, elevator, arm, climber))
            ).finallyDo(Runnable { state.push(task = AutoTask.IDLE) })
        )
        ioPrint("POV_DOWN ")
        driveController.povUp().onTrue(justScore())
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
            nextPosition = Constants.SuperstructureConstants.SuperstructureState.L2
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.y().onTrue(runOnce({
            nextPosition = Constants.SuperstructureConstants.SuperstructureState.L3
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.x().onTrue(runOnce({
            nextPosition = Constants.SuperstructureConstants.SuperstructureState.L4
        }).ignoringDisable(true).onlyIf(::hmiEnabled))
        hmi.a().onTrue(runOnce({ keepGoing = true }).onlyIf(::hmiEnabled))
            .onFalse(runOnce({ keepGoing = false }).onlyIf(::hmiEnabled))
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
                grabAlgaeToggle = false
            } else {
                grabAlgaeToggle = true
                nextAlgaePosition = when (joystickValue) {
                    1 -> PathfindingFactories.Reef.AB_ALGAE
                    2 -> PathfindingFactories.Reef.CD_ALGAE
                    3 -> PathfindingFactories.Reef.EF_ALGAE
                    4 -> PathfindingFactories.Reef.GH_ALGAE
                    5 -> PathfindingFactories.Reef.IJ_ALGAE
                    else -> PathfindingFactories.Reef.KL_ALGAE
                }
            }
        }
        state.updateIO()
    }

    private fun updateHmi() {
        if (hmi.povUp().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.G
                false -> PathfindingFactories.Reef.H
            }
        }
        if (hmi.povUpRight().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.E
                false -> PathfindingFactories.Reef.F
            }
        }
        if (hmi.povDownRight().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.C
                false -> PathfindingFactories.Reef.D
            }
        }
        if (hmi.povDown().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.A
                false -> PathfindingFactories.Reef.B
            }
        }
        if (hmi.povDownLeft().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.K
                false -> PathfindingFactories.Reef.L
            }
        }
        if (hmi.povUpLeft().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.I
                false -> PathfindingFactories.Reef.J
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
        stationSide: PathfindingFactories.CoralStationSide,
        initialReef: PathfindingFactories.Reef,
        vararg reefs: PathfindingFactories.Reef
    ): Command {
        var auto: Command = AutoBuilder.buildAuto(
            "Start at $bargeAdjective ${
                when (stationSide) {
                    PathfindingFactories.CoralStationSide.LEFT -> "Left"
                    PathfindingFactories.CoralStationSide.RIGHT -> "Right"
                    PathfindingFactories.CoralStationSide.NONE -> throw NotImplementedError()
                }
            } and Score Coral ${
                when (stationSide) {
                    PathfindingFactories.CoralStationSide.LEFT -> "Left"
                    PathfindingFactories.CoralStationSide.RIGHT -> "Right"
                    PathfindingFactories.CoralStationSide.NONE -> throw NotImplementedError()
                }
            } But End Halfway Through"
        ).withTimeout(5.0).andThen({
            updateReef(initialReef)
            nextPosition = Constants.SuperstructureConstants.SuperstructureState.L4
            nextStation = stationSide
        })

        for (reef in reefs.copyOfRange(0, reefs.size - 1)) {
            auto = auto.andThen(
                runOneFullCoralCycleButWaitTime().alongWith(
                    waitSeconds(0.75).andThen(
                        { updateReef(reef) })
                )
            )
        }

        auto = auto.andThen({ nextReef = reefs.last() }).andThen(runOneFullCoralCycleButWaitTime())
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
                PathfindingFactories.CoralStationSide.LEFT,
                PathfindingFactories.Reef.J,
                PathfindingFactories.Reef.K,
                PathfindingFactories.Reef.L,
                PathfindingFactories.Reef.A
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Far Left, Left, Home Field)", createPathfindingAuto(
                "Far",
                PathfindingFactories.CoralStationSide.LEFT,
                PathfindingFactories.Reef.J,
                PathfindingFactories.Reef.A,
                PathfindingFactories.Reef.B
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Left, All)", createPathfindingAuto(
                "Mid",
                PathfindingFactories.CoralStationSide.LEFT,
                PathfindingFactories.Reef.J,
                PathfindingFactories.Reef.K,
                PathfindingFactories.Reef.L,
                PathfindingFactories.Reef.A
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Left, Home Field)", createPathfindingAuto(
                "Mid",
                PathfindingFactories.CoralStationSide.LEFT,
                PathfindingFactories.Reef.J,
                PathfindingFactories.Reef.A,
                PathfindingFactories.Reef.B
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Far Right, Right, All)", createPathfindingAuto(
                "Far",
                PathfindingFactories.CoralStationSide.RIGHT,
                PathfindingFactories.Reef.E,
                PathfindingFactories.Reef.D,
                PathfindingFactories.Reef.C,
                PathfindingFactories.Reef.B,
                PathfindingFactories.Reef.A
            )
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Right, Right, All)", createPathfindingAuto(
                "Mid",
                PathfindingFactories.CoralStationSide.RIGHT,
                PathfindingFactories.Reef.E,
                PathfindingFactories.Reef.D,
                PathfindingFactories.Reef.C,
                PathfindingFactories.Reef.B,
                PathfindingFactories.Reef.A
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
        SmartDashboard.putData("Log Drive Time", runOnce({ drive.logCycleTime = true }).ignoringDisable(true))
        SmartDashboard.putData(
            elevator.homeElevator().deadlineFor(arm.moveArmToAngleWithoutEnding(90.0)).withName("Home Elevator")
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
            runOnce({ nextReef = PathfindingFactories.Reef.A }).withName("\nSelect Reef A\n")
        )
        // repeat for other reefs
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.B }).withName("\nSelect Reef B\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.C }).withName("\nSelect Reef C\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.D }).withName("\nSelect Reef D\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.E }).withName("\nSelect Reef E\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.F }).withName("\nSelect Reef F\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.G }).withName("\nSelect Reef G\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.H }).withName("\nSelect Reef H\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.I }).withName("\nSelect Reef I\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.J }).withName("\nSelect Reef J\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.K }).withName("\nSelect Reef K\n")
        )
        SmartDashboard.putData(
            runOnce({ nextReef = PathfindingFactories.Reef.L }).withName("\nSelect Reef L\n")
        )

        // algae as well
        SmartDashboard.putData(
            runOnce({
                nextAlgaePosition = PathfindingFactories.Reef.AB_ALGAE; grabAlgaeToggle = true
            }).withName("\nSelect Algae AB\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextAlgaePosition = PathfindingFactories.Reef.CD_ALGAE; grabAlgaeToggle = true
            }).withName("\nSelect Algae CD\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextAlgaePosition = PathfindingFactories.Reef.EF_ALGAE; grabAlgaeToggle = true
            }).withName("\nSelect Algae EF\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextAlgaePosition = PathfindingFactories.Reef.GH_ALGAE; grabAlgaeToggle = true
            }).withName("\nSelect Algae GH\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextAlgaePosition = PathfindingFactories.Reef.IJ_ALGAE; grabAlgaeToggle = true
            }).withName("\nSelect Algae IJ\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextAlgaePosition = PathfindingFactories.Reef.KL_ALGAE; grabAlgaeToggle = true
            }).withName("\nSelect Algae KL\n")
        )
        SmartDashboard.putData(
            runOnce({ grabAlgaeToggle = false }).withName("\nDisable Algae\n")
        )

        // finally, positions
        SmartDashboard.putData(
            runOnce({
                nextPosition = Constants.SuperstructureConstants.SuperstructureState.L1
            }).withName("\nSelect Position L1\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextPosition = Constants.SuperstructureConstants.SuperstructureState.L2
            }).withName("\nSelect Position L2\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextPosition = Constants.SuperstructureConstants.SuperstructureState.L3
            }).withName("\nSelect Position L3\n")
        )
        SmartDashboard.putData(
            runOnce({
                nextPosition = Constants.SuperstructureConstants.SuperstructureState.L4
            }).withName("\nSelect Position L4\n")
        )

        SmartDashboard.putData(
            Commands.run({ keepGoing = true }).finallyDo(Runnable { keepGoing = false })
                .withName("\nKeep Going\n")
        )

    }


    private fun addNamedCommands() {
        NamedCommands.registerCommand(
            "home", superstructureCommands.home(elevator, arm, climber)
        )
        NamedCommands.registerCommand(
            "L4", superstructureCommands.l4WithoutSafety(elevator, arm)
        )
        NamedCommands.registerCommand(
            "deposit", superstructureCommands.scoreAtPositionWithoutDrive(
                elevator, arm, clawIntake, Constants.SuperstructureConstants.SuperstructureState.L4
            )
        )
        NamedCommands.registerCommand(
            "Pre-Coral Pickup", arm.moveArmToAngle(130.0).alongWith(elevator.goToPosition(37.0))
        )
        NamedCommands.registerCommand(
            "Fix Pivot", elevator.goToPosition(40.0).deadlineFor(climber.moveClimberOpenLoop({ 0.0 }, { 0.0 })).andThen(
                superstructureCommands.algaeIntakeWithoutSafety(
                    elevator, arm, climber
                )
            )
        )
        NamedCommands.registerCommand(
            "Fix Pivot and L4", climber.moveClimberOpenLoop({ 0.0 }, { 0.0 }).withDeadline(
                superstructureCommands.l4WithoutSafety(
                    elevator, arm
                )
            ).andThen(
                climber.moveClimberToIntakePosition().withDeadline(waitSeconds(0.25))
            )
        )
        NamedCommands.registerCommand(
            "Pick Up and L4", superstructureCommands.pickUpCoral(
                elevator, arm, clawIntake, climber
            ).raceWith(waitSeconds(3.75)).andThen(
                superstructureCommands.l4WithoutSafety(
                    elevator, arm
                )
            )
        )
        NamedCommands.registerCommand(
            "Low Algae", superstructureCommands.goToPosition(
                elevator, arm, climber, Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE
            ).alongWith(
                clawIntake.intakeWithoutStoppingForAlgae().withName("Run Intake (auto-added)")
            )
        )
        NamedCommands.registerCommand(
            "Spit Out", clawIntake.outtakeFaster()
        )
        NamedCommands.registerCommand(
            "Barge Algae", superstructureCommands.goToPosition(
                elevator, arm, climber, Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH
            )
        )
        NamedCommands.registerCommand(
            "Lock Wheels", run({ drive.stopWithX() }, drive)
        )
        NamedCommands.registerCommand(
            "Run Intake", clawIntake.intakeWithoutStoppingForAlgae()
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
        if (climber.position > 90.0) elevator.goToPosition(40.0)
            .alongWith(arm.moveArmToAngle(Constants.SuperstructureConstants.ArmConstants.ArmState.PRE_CORAL_PICKUP))
            .deadlineFor(climber.moveClimberOpenLoop({ 0.0 }, { 0.0 }))
            .andThen(climber.moveClimberToIntakePosition().withTimeout(1.0)).andThen(
                superstructureCommands.preCoralPickup(
                    elevator, arm, climber
                ).withTimeout(2.0)
            ).withName("Fix Arm-Pivot System").schedule()
    }

    fun disableAuto() {
        coralStatus = CoralStatus.NONE
    }
    // </editor-fold>

    // <editor-fold desc="Auto functions">

    // <editor-fold desc="Full Cycles">
    private fun runOneFullCoralCycle(): Command {
        var inPosition = false
        // go to coral station; requires drive, arm, elevator, and climber
        return sequence(
            goToCoralStation(),
            // wait until the driver signals a coral on the intake (we have no way of detecting this)
            lockWheelsAndWaitForInput(), runOnce({ state.push(task = AutoTask.TO_REEF) }),
            // pathfind to reef; requires drive, elevator, arm, intake, climber
            parallel(
                // this command sequence concerns the drivebase + pathfinding
                sequence(
                    // pathfinding speed; doesn't require anything
                    runOnce({
                        drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.coralIntakeSpeed)
                        inPosition = false
                    }),


                    // actually go to the reef
                    pathfindToReef().alongWith(
                        // wait until the claw has the coral secured;
                        // if we move too fast,
                        // we risk throwing it off the intake,
                        // but we still want to be able to move quickly
                        waitUntil { coralStatus == CoralStatus.IN_CLAW }.andThen({
                            // set pathfinding speed to the normal speed
                            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
                        })
                    ),

                    // at this point we're in the reef protected zone
                    // which should provide reprieve
                    // if the opposing alliance is playing defense
                    // and knocks us out of alignment.
                    // more importantly, we're stopped and waiting

                    // wait until the superstructure is in the position
                    // we want it to be in for scoring
                    waitUntil { inPosition }
                        // while we wait, lock the wheels
                        // this might actually help
                        // mitigate some of the "getting knocked out of alignment"
                        // issues we could face in a real match
                        .deadlineFor(run({ drive.stopWithX() }, drive)).andThen(
                            // close that last bit of distance
                            finalReefLineup()
                        ).withName("Pathfind to Reef (final)")
                ), // end drivebase sequence

                // wait until the claw has the coral secured and set that state
                waitUntil { clawIntake.grabbed }.andThen({ coralStatus = CoralStatus.IN_CLAW }),

                // superstructure stuff
                sequence(
                    // pick up the coral from the intake
                    superstructureCommands.pickUpCoral(
                        elevator, arm, clawIntake, climber
                    )
                        // i couldn't imagine a situation
                        // in which the coral wouldn't be on the intake,
                        // because this command can already only start
                        // once the driver confirms it,
                        // but this check could be useful at some point or another
                        .onlyIf { coralStatus == CoralStatus.ON_INTAKE },

                    // might fix an issue we were having
                    // where the arm tries to go to some random position
                    runOnce({ arm.setGoalToCurrent() }),

                    // wait until we're near where we need to be
                    waitUntil(drive::nearGoal).andThen(
                        // defer this command construction until it's called.
                        // that way,
                        // we're checking what position
                        // the drivers want the superstructure
                        // to be in at the last possible moment
                        // in case they change something.
                        // the field is dynamic, after all
                        runOnce({ updateHmi() })
                    ).andThen(
                        defer({
                            // go to the position
                            superstructureCommands.goToPositionWithoutSafety(
                                elevator, arm, climber, nextPosition
                            )
                        }, setOf(elevator, arm, climber))
                    ),
                    // once we're in position,
                    // set the state so the drivebase can continue its final lineup
                    runOnce({ inPosition = true })
                )
            ), // end pathfinding to reef and lining up and all that jazz


            // deferred command, for the same reasons as the other one
            defer({
                // score the coral
                superstructureCommands.scoreAtPositionFaster(
                    elevator, arm, clawIntake, drive, nextPosition
                )
            }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                // set state of the coral
                Runnable { coralStatus = CoralStatus.NONE })
                // only do this deferred command + compositions
                // if we actually have the coral
                .onlyIf { coralStatus == CoralStatus.IN_CLAW }).andThen(
            select(
                mapOf(
                    Pair(
                        true, runOnce({ runOneFullCoralCycle().schedule() })
                    ), Pair(
                        false, none()
                    )
                ), ::keepGoing
            )
        ).finallyDo(Runnable { state.push(task = AutoTask.IDLE) }).withName("Full Coral Cycle")

    }

    private fun justScore(): Command {
        var inPosition = false
        return parallel(
            runOnce({ state.push(task = AutoTask.TO_REEF) }),
            // this command sequence concerns the drivebase + pathfinding
            sequence(
                // pathfinding speed; doesn't require anything
                runOnce({
                    drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
                    inPosition = false
                }),


                // actually go to the reef
                pathfindToReef(),

                // at this point we're in the reef protected zone
                // which should provide reprieve
                // if the opposing alliance is playing defense
                // and knocks us out of alignment.
                // more importantly, we're stopped and waiting

                // wait until the superstructure is in the position
                // we want it to be in for scoring
                waitUntil { inPosition }
                    // while we wait, lock the wheels
                    // this might actually help
                    // mitigate some of the "getting knocked out of alignment"
                    // issues we could face in a real match
                    .deadlineFor(run({ drive.stopWithX() }, drive)).andThen(
                        // close that last bit of distance
                        finalReefLineup()
                    ).withName("Pathfind to Reef (final)")
            ), // end drivebase sequence

            // superstructure stuff
            sequence(
                // might fix an issue we were having
                // where the arm tries to go to some random position
                runOnce({ arm.setGoalToCurrent() }),

                // wait until we're near where we need to be
                waitUntil(drive::nearGoal).andThen(
                    // defer this command construction until it's called.
                    // that way,
                    // we're checking what position
                    // the drivers want the superstructure
                    // to be in at the last possible moment
                    // in case they change something.
                    // the field is dynamic, after all
                    defer({
                        // go to the position
                        superstructureCommands.goToPositionWithoutSafety(
                            elevator, arm, climber, nextPosition
                        )
                    }, setOf(elevator, arm, climber))
                ),
                // once we're in position,
                // set the state so the drivebase can continue its final lineup
                runOnce({ inPosition = true })
            )
        ) // end pathfinding to reef and lining up and all that jazz

            .andThen(
                // deferred command, for the same reasons as the other one
                defer({
                    // score the coral
                    superstructureCommands.scoreAtPositionFaster(
                        elevator, arm, clawIntake, drive, nextPosition
                    )
                }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                    // set state of the coral
                    Runnable { coralStatus = CoralStatus.NONE })
            ).finallyDo(Runnable { state.push(task = AutoTask.IDLE) }).withName("Score Coral")

    }

    private fun runOneFullCoralCycleButWaitTime(): Command {
        var inPosition = false
        // go to coral station; requires drive, arm, elevator, and climber
        return sequence(
            goToCoralStation(),

            // wait for some arbitrary amount of time defined elsewhere
            lockWheelsAndWaitTime(), runOnce({ state.push(task = AutoTask.TO_REEF) }),

            // pathfind to reef; requires drive, elevator, arm, intake, climber
            parallel(

                // this command sequence concerns the drivebase + pathfinding
                sequence(
                    // pathfinding speed; doesn't require anything
                    runOnce({
                        drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.coralIntakeSpeed)
                        inPosition = false
                    }),


                    // actually go to the reef
                    finalReefLineup(), run({ drive.stopWithX() }, drive).withTimeout(0.1)
                ), // end drivebase sequence

                // wait until the claw has the coral secured and set that state
                waitUntil { clawIntake.grabbed }.andThen({
                    coralStatus = CoralStatus.IN_CLAW
                    drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
                }),

                // superstructure stuff
                sequence(
                    // pick up the coral from the intake
                    superstructureCommands.pickUpCoral(
                        elevator, arm, clawIntake, climber
                    )
                        // i couldn't imagine a situation
                        // in which the coral wouldn't be on the intake,
                        // because this command can already only start
                        // once the driver confirms it,
                        // but this check could be useful at some point or another
                        .onlyIf { coralStatus == CoralStatus.ON_INTAKE },

                    // might fix an issue we were having
                    // where the arm tries to go to some random position
                    runOnce({ arm.setGoalToCurrent() }),

                    // wait until we're near where we need to be
                    // defer this command construction until it's called.
                    // that way,
                    // we're checking what position
                    // the drivers want the superstructure
                    // to be in at the last possible moment
                    // in case they change something.
                    // the field is dynamic, after all
                    defer({
                        // go to the position
                        superstructureCommands.goToPositionWithoutSafety(
                            elevator, arm, climber, nextPosition
                        )
                    }, setOf(elevator, arm, climber)),
                    // once we're in position,
                    // set the state so the drivebase can continue its final lineup
                    runOnce({ inPosition = true })
                )
            ), // end pathfinding to reef and lining up and all that jazz

            // deferred command, for the same reasons as the other one
            defer({
                // score the coral
                superstructureCommands.scoreAtPositionFaster(
                    elevator, arm, clawIntake, drive, nextPosition
                )
            }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                // set state of the coral
                Runnable { coralStatus = CoralStatus.NONE })
                // only do this deferred command + compositions
                // if we actually have the coral
                .onlyIf { coralStatus == CoralStatus.IN_CLAW }).finallyDo(Runnable { state.push(task = AutoTask.IDLE) })
            .withName("[AUTO] Full Coral Cycle")
    }

    private fun algaeCycle(): Command {

        return runOnce({
            // check if we have anything selected for barge positions.
            // if not, default to none
            bargePosition = bargeChooser.get() ?: BargePosition.NONE
            state.push(task = AutoTask.TO_ALGAE)
        }).andThen(
            sequence(
                sequence(
                    sequence(
                        defer(
                            {
                                // pathfind to where we need to be,
                                // just 20 inches back so we have room to swing the arm around
                                PathfindingFactories.pathfindToReefButBackALittleMore(
                                    drive, { nextAlgaePosition }, driveTranslationalControlSupplier
                                )
                            }, setOf(drive)
                        ),
                        // wait until we're near the target reef,
                        // then put the superstructure in whatever position we need to be in to grab algae

                        defer(
                            {
                                superstructureCommands.goToPosition(
                                    elevator, arm, climber, when (nextAlgaePosition) {
                                        PathfindingFactories.Reef.AB_ALGAE, PathfindingFactories.Reef.EF_ALGAE, PathfindingFactories.Reef.IJ_ALGAE -> Constants.SuperstructureConstants.SuperstructureState.UPPER_REEF_ALGAE
                                        else -> Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE
                                    }
                                )
                            }, setOf(arm, elevator, climber)
                        ).deadlineFor(
                            run({ drive.stopWithX() }, drive)
                        )
                    ),

                    // set the pathfinding speed
                    // to be slower so we're not slamming into the reef at top speed
                    runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.algaeGrabSpeed) }),

                    // pathfind forward so we can actually pick the algae up
                    PathfindingFactories.pathfindToReefButBackALittleLess(
                        drive, { nextAlgaePosition }, driveTranslationalControlSupplier
                    ).deadlineFor(
                        // pin this onto the end
                        // maybe it'll save us a little time?
                        clawIntake.intakeWithoutStoppingForAlgae()
                    ),

                    // lock the wheels
                    run(
                        { drive.stopWithX() }, drive
                    ).withDeadline(
                        // spin the intake
                        clawIntake.intakeWithoutStoppingForAlgae().withDeadline(
                            // wait until the motor stalls
                            // (surefire way to tell if we have an algae) and then wait an extra half-second
                            // FIXME: check if this value can go lower
                            // update 8/5/25: turned down by half
                            waitUntil(clawIntake::grabbed).andThen(waitSeconds(0.25))
                            // set algae state
                        ).andThen({ algaeStatus = AlgaeStatus.IN_CLAW })
                    ),
                    // back up
                    drive.backUp()
                ).onlyIf { algaeStatus == AlgaeStatus.NONE }, // only do this if we don't have algae
                // decide what to do based on the barge position selection
                select(
                    mapOf(
                        BargePosition.NONE to spitOutAlgae(),
                        BargePosition.LEFT to putAlgaeInBarge(),
                        BargePosition.MIDDLE to putAlgaeInBarge(),
                        BargePosition.RIGHT to putAlgaeInBarge()
                    )
                ) { bargePosition }.onlyIf { algaeStatus == AlgaeStatus.IN_CLAW } // use the barge position as the key
            )).finallyDo(Runnable { state.push(task = AutoTask.IDLE) })
    }

    // </editor-fold>
    // <editor-fold desc="Larger utility functions">

    private fun putAlgaeInBarge(): Command {
        return sequence(
            runOnce({ state.push(task = AutoTask.TO_BARGE) }),
            // set pathfinding speed to whatever we have set for going to the barge
            runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toBargeSpeed) }),

            // pathfind to the barge
            defer({
                PathfindingFactories.pathfindToPosition(
                    drive, Constants.PathfindingConstants.getPosition(bargePosition), driveTranslationalControlSupplier
                )
            }, setOf(drive)).alongWith(
                // wait until we're near the barge
                waitSeconds(0.5).andThen(waitUntil(drive::lessNearGoal)).andThen(
                    // and then put the superstructure in the right position
                    superstructureCommands.goToPosition(
                        elevator, arm, climber, Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH
                    )
                )
                // while this is happening, spin the intake so we can keep the algae in
            ).deadlineFor(clawIntake.intakeWithoutStoppingForAlgae()),

            // drive forward while spitting the algae out.
            // the drive command ends after half a second
            drive.goForward().deadlineFor(
                clawIntake.outtakeMaxSpeed()
            ).andThen(
                // stop the intake and set algae status to none
                runOnce({
                    algaeStatus = AlgaeStatus.NONE
                }).alongWith(clawIntake.stopOnce())
            ),

            // back up a little bit
            drive.backUp(),

            // stop the drive
            // while we put the superstructure back into the position it needs to be in
            superstructureCommands.preCoralPickup(elevator, arm, climber).deadlineFor(
                run({ drive.stopWithX() }, drive)
            ).withTimeout(3.0)
        ).finallyDo(Runnable {
            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed); state.push(
            task = AutoTask.IDLE
        )
        }).withName("Put Algae in Barge")
    }

    private fun spitOutAlgae(): Command {
        return sequence(
            runOnce({ state.push(task = AutoTask.SPIT_OUT_ALGAE) }),
            // rotate 180°, but stop after half a second
            DriveCommands.joystickDriveAtAngle(
                drive,
                { 0.0 },
                { 0.0 },
                { drive.pose.rotation.rotateBy(Rotation2d(Math.PI)) }).withDeadline(waitSeconds(0.5)),
            // spin the intake to spit out algae
            // no way to determine when this is done so just do it for 3/4 of a second
            run({ drive.stopWithX() }, drive).alongWith(
                clawIntake.outtakeFaster()
            ).withDeadline(
                waitSeconds(0.75)
            ),
            // set algae status to none, stop intake
            runOnce({
                algaeStatus = AlgaeStatus.NONE
            }).alongWith(clawIntake.stopOnce())
        ).finallyDo(Runnable {
            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed); state.push(
            task = AutoTask.IDLE
        )
        })
    }
    // </editor-fold>
    // <editor-fold desc="Low-level functions">

    private fun lockWheelsAndWaitForInput(): Command = DriveCommands.joystickDrive(
        drive,
        { -driveController.leftY },
        { -driveController.leftX },
        { -driveController.rightX }).withDeadline(
        waitUntil { coralStatus == CoralStatus.ON_INTAKE })
        .alongWith(runOnce({ state.push(task = AutoTask.WAITING) }).finallyDo(Runnable { state.push(task = AutoTask.IDLE) }))
        .withName("Lock Wheels")


    private fun lockWheelsAndWaitTime(): Command = run(
        { drive.stopWithX() }, drive
    ).withDeadline(
        waitSeconds(Constants.PathfindingConstants.benCompensation).andThen({
            coralStatus = CoralStatus.ON_INTAKE
        }).alongWith(runOnce({ state.push(task = AutoTask.WAITING) }))
            .finallyDo(Runnable { state.push(task = AutoTask.IDLE) }).withName("Lock Wheels")
    )

    private fun finalReefLineup(): Command = defer({
        PathfindingFactories.pathfindToReef(
            drive, { nextReef }, driveTranslationalControlSupplier
        )
    }, setOf(drive)).withName("Pathfind to Reef")

    private fun pathfindToReef(): Command = defer({
        PathfindingFactories.pathfindToReefButBackALittle(
            drive, { nextReef }, driveTranslationalControlSupplier
        )
    }, setOf(drive))

    private fun goToCoralStation(): Command =
        runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toCoralStationSpeed); state.push(task = AutoTask.TO_CORAL_STATION) }).andThen(
            defer(
                {
                    PathfindingFactories.pathfindToCoralStation(
                        drive, { nextStation }, driveTranslationalControlSupplier
                    )
                }, setOf(drive)
            ).deadlineFor(
                superstructureCommands.preCoralPickup(elevator, arm, climber)
            )
        ).finallyDo(Runnable {
            arm.setGoalToCurrent(); drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
            state.push(task = AutoTask.IDLE)
        }).onlyIf { coralStatus == CoralStatus.NONE }.withName("Pathfind to Coral Station")

    private fun updateReef(reef: PathfindingFactories.Reef) {
        nextReef = reef
    }
    // </editor-fold>
    // </editor-fold>
}


// Algae format for HMI input goes AB reef first, counterclockwise
