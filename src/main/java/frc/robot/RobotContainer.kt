package frc.robot

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.Autos
import frc.robot.commands.ExampleCommand
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.CommandSwerveDrivetrain
import frc.robot.subsystems.ExampleSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    private val MaxSpeed =
        TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond) // kSpeedAt12Volts desired top speed
    private val MaxAngularRate = Units.RotationsPerSecond.of(0.75)
        .`in`(Units.RadiansPerSecond) // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private val drive: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
    private val brake = SwerveRequest.SwerveDriveBrake()
    private val point = SwerveRequest.PointWheelsAt()

    private val logger = Telemetry(MaxSpeed)

    private val joystick = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)

    val drivetrain: CommandSwerveDrivetrain = TunerConstants.createDrivetrain()
        
    init
    {
        configureBindings()
        // Reference the Autos object so that it is initialized, placing the chooser on the dashboard
        Autos
    }

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings()
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.defaultCommand = drivetrain.applyRequest {
            drive.withVelocityX(-joystick.leftY * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.leftX * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.rightX * MaxAngularRate)
        } // Drive counterclockwise with negative X (left)


        joystick.a().whileTrue(drivetrain.applyRequest { brake })
        joystick.b().whileTrue(drivetrain.applyRequest {
            point.withModuleDirection(
                Rotation2d(
                    -joystick.leftY,
                    -joystick.leftX
                )
            )
        })

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y())
            .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
        joystick.back().and(joystick.x())
            .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        joystick.start().and(joystick.y())
            .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        joystick.start().and(joystick.x())
            .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce { drivetrain.seedFieldCentric() })

        drivetrain.registerTelemetry { state: SwerveDriveState -> logger.telemeterize(state) }
    }
}