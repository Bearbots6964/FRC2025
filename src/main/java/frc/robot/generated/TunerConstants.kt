package frc.robot.generated

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import frc.robot.subsystems.CommandSwerveDrivetrain

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
object TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private val steerGains: Slot0Configs = Slot0Configs()
        .withKP(100.0).withKI(0.0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0.0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private val driveGains: Slot0Configs = Slot0Configs()
        .withKP(0.1).withKI(0.0).withKD(0.0)
        .withKS(0.0).withKV(0.124)

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private val steerClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private val driveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The type of motor used for the drive motor
    private val driveMotorType = DriveMotorArrangement.TalonFX_Integrated

    // The type of motor used for the drive motor
    private val steerMotorType = SteerMotorArrangement.TalonFX_Integrated

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private val steerFeedbackType = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private val slipCurrent: Current = Units.Amps.of(40.0)

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private val driveInitialConfigs = TalonFXConfiguration()
    private val steerInitialConfigs: TalonFXConfiguration = TalonFXConfiguration()
        .withCurrentLimits(
            CurrentLimitsConfigs() // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Units.Amps.of(60.0))
                .withStatorCurrentLimitEnable(true)
        )
    private val encoderInitialConfigs = CANcoderConfiguration()

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private val pigeonConfigs: Pigeon2Configuration? = null

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    val canBus: CANBus = CANBus("Drivebase", "./logs/example.hoot")

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    val speedAt12Volts: LinearVelocity = Units.FeetPerSecond.of(12.5)

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private const val coupleRatio = 3.5714285714285716

    private const val driveGearRatio = 6.746031746031747
    private const val steerGearRatio = 21.428571428571427
    private val kWheelRadius: Distance = Units.Inches.of(2.0)

    private const val invertLeftSide = false
    private const val invertRightSide = true

    private const val pigeonId = 14

    // These are only used for simulation
    private val steerInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)
    private val driveInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)

    // Simulated voltage necessary to overcome friction
    private val steerFrictionVoltage: Voltage = Units.Volts.of(0.2)
    private val driveFrictionVoltage: Voltage = Units.Volts.of(0.2)

    private val DrivetrainConstants: SwerveDrivetrainConstants = SwerveDrivetrainConstants()
        .withCANBusName(canBus.name)
        .withPigeon2Id(pigeonId)
        .withPigeon2Configs(pigeonConfigs)

    private val ConstantCreator: SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(driveGearRatio)
            .withSteerMotorGearRatio(steerGearRatio)
            .withCouplingGearRatio(coupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSlipCurrent(slipCurrent)
            .withSpeedAt12Volts(speedAt12Volts)
            .withDriveMotorType(driveMotorType)
            .withSteerMotorType(steerMotorType)
            .withFeedbackSource(steerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(steerInertia)
            .withDriveInertia(driveInertia)
            .withSteerFrictionVoltage(steerFrictionVoltage)
            .withDriveFrictionVoltage(driveFrictionVoltage)


    // Front Left
    private const val frontLeftDriveMotorId = 4
    private const val frontLeftSteerMotorId = 6
    private const val frontLeftEncoderId = 5
    private val frontLeftEncoderOffset: Angle = Units.Rotations.of(-0.354736328125)
    private const val frontLeftSteerMotorInverted = true
    private const val frontLeftEncoderInverted = false

    private val frontLeftXPos: Distance = Units.Inches.of(12.125)
    private val frontLeftYPos: Distance = Units.Inches.of(12.125)

    // Front Right
    private const val frontRightDriveMotorId = 1
    private const val frontRightSteerMotorId = 3
    private const val frontRightEncoderId = 2
    private val frontRightEncoderOffset: Angle = Units.Rotations.of(-0.321044921875)
    private const val frontRightSteerMotorInverted = true
    private const val frontRightEncoderInverted = false

    private val kFrontRightXPos: Distance = Units.Inches.of(12.125)
    private val kFrontRightYPos: Distance = Units.Inches.of(-12.125)

    // Back Left
    private const val backLeftDriveMotorId = 7
    private const val backLeftSteerMotorId = 9
    private const val backLeftEncoderId = 8
    private val backLeftEncoderOffset: Angle = Units.Rotations.of(0.011474609375)
    private const val backLeftSteerMotorInverted = true
    private const val backLeftEncoderInverted = false

    private val backLeftXPos: Distance = Units.Inches.of(-12.125)
    private val backLeftYPos: Distance = Units.Inches.of(12.125)

    // Back Right
    private const val backRightDriveMotorId = 10
    private const val backRightSteerMotorId = 12
    private const val backRightEncoderId = 11
    private val backRightEncoderOffset: Angle = Units.Rotations.of(0.31103515625)
    private const val backRightSteerMotorInverted = true
    private const val backRightEncoderInverted = false

    private val backRightXPos: Distance = Units.Inches.of(-12.125)
    private val backRightYPos: Distance = Units.Inches.of(-12.125)


    val FrontLeft: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            frontLeftSteerMotorId,
            frontLeftDriveMotorId,
            frontLeftEncoderId,
            frontLeftEncoderOffset,
            frontLeftXPos,
            frontLeftYPos,
            invertLeftSide,
            frontLeftSteerMotorInverted,
            frontLeftEncoderInverted
        )
    val FrontRight: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            frontRightSteerMotorId,
            frontRightDriveMotorId,
            frontRightEncoderId,
            frontRightEncoderOffset,
            kFrontRightXPos,
            kFrontRightYPos,
            invertRightSide,
            frontRightSteerMotorInverted,
            frontRightEncoderInverted
        )
    val BackLeft: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            backLeftSteerMotorId,
            backLeftDriveMotorId,
            backLeftEncoderId,
            backLeftEncoderOffset,
            backLeftXPos,
            backLeftYPos,
            invertLeftSide,
            backLeftSteerMotorInverted,
            backLeftEncoderInverted
        )
    val BackRight: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            backRightSteerMotorId,
            backRightDriveMotorId,
            backRightEncoderId,
            backRightEncoderOffset,
            backRightXPos,
            backRightYPos,
            invertRightSide,
            backRightSteerMotorInverted,
            backRightEncoderInverted
        )

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    fun createDrivetrain(): CommandSwerveDrivetrain {
        return CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        )
    }


    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    open class TunerSwerveDrivetrain : SwerveDrivetrain<TalonFX?, TalonFX?, CANcoder?> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         *
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        constructor(
            drivetrainConstants: SwerveDrivetrainConstants,
            vararg modules: SwerveModuleConstants<*, *, *>?
        ) : super(
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<CANcoder?> { deviceId: Int, canbus: String? ->
                CANcoder(
                    deviceId,
                    canbus
                )
            },
            drivetrainConstants, *modules
        )

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         *
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         * unspecified or set to 0 Hz, this is 250 Hz on
         * CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        constructor(
            drivetrainConstants: SwerveDrivetrainConstants,
            odometryUpdateFrequency: Double,
            vararg modules: SwerveModuleConstants<*, *, *>?
        ) : super(
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<CANcoder?> { deviceId: Int, canbus: String? ->
                CANcoder(
                    deviceId,
                    canbus
                )
            },
            drivetrainConstants, odometryUpdateFrequency, *modules
        )

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         *
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         * unspecified or set to 0 Hz, this is 250 Hz on
         * CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         * in the form [x, y, theta]ᵀ, with units in meters
         * and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         * in the form [x, y, theta]ᵀ, with units in meters
         * and radians
         * @param modules                   Constants for each specific module
         */
        constructor(
            drivetrainConstants: SwerveDrivetrainConstants,
            odometryUpdateFrequency: Double,
            odometryStandardDeviation: Matrix<N3?, N1?>,
            visionStandardDeviation: Matrix<N3?, N1?>,
            vararg modules: SwerveModuleConstants<*, *, *>?
        ) : super(
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<CANcoder?> { deviceId: Int, canbus: String? ->
                CANcoder(
                    deviceId,
                    canbus
                )
            },
            drivetrainConstants, odometryUpdateFrequency,
            odometryStandardDeviation, visionStandardDeviation, *modules
        )
    }
}
