package frc.robot

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit

class Telemetry(private val MaxSpeed: Double) {
    /* What to publish over networktables for telemetry */
    private val inst: NetworkTableInstance = NetworkTableInstance.getDefault()

    /* Robot swerve drive state */
    private val driveStateTable: NetworkTable = inst.getTable("DriveState")
    private val drivePose: StructPublisher<Pose2d> =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish()
    private val driveSpeeds: StructPublisher<ChassisSpeeds> =
        driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish()
    private val driveModuleStates: StructArrayPublisher<SwerveModuleState> =
        driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish()
    private val driveModuleTargets: StructArrayPublisher<SwerveModuleState> =
        driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish()
    private val driveModulePositions: StructArrayPublisher<SwerveModulePosition> =
        driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
            .publish()
    private val driveTimestamp: DoublePublisher =
        driveStateTable.getDoubleTopic("Timestamp").publish()
    private val driveOdometryFrequency: DoublePublisher =
        driveStateTable.getDoubleTopic("OdometryFrequency").publish()

    /* Robot pose for field positioning */
    private val table: NetworkTable = inst.getTable("Pose")
    private val fieldPub: DoubleArrayPublisher = table.getDoubleArrayTopic("robotPose").publish()
    private val fieldTypePub: StringPublisher = table.getStringTopic(".type").publish()

    /* Mechanisms to represent the swerve module states */
    private val moduleMechanisms = arrayOf(
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
    )

    /* A direction and length changing ligament for speed representation */
    private val moduleSpeeds = arrayOf(
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
    )

    /* A direction changing and length constant ligament for module direction */
    private val moduleDirections = arrayOf(
        moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
    )

    private val poseArray = DoubleArray(3)
    private val moduleStatesArray = DoubleArray(8)
    private val moduleTargetsArray = DoubleArray(8)

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    init {
        SignalLogger.start()
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.  */
    fun telemeterize(state: SwerveDriveState) {/* Telemeterize the swerve drive state */
        drivePose.set(state.Pose)
        driveSpeeds.set(state.Speeds)
        driveModuleStates.set(state.ModuleStates)
        driveModuleTargets.set(state.ModuleTargets)
        driveModulePositions.set(state.ModulePositions)
        driveTimestamp.set(state.Timestamp)
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod)

        /* Also write to log file */
        poseArray[0] = state.Pose.x
        poseArray[1] = state.Pose.y
        poseArray[2] = state.Pose.rotation.degrees
        for (i in 0..3) {
            moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.radians
            moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond
            moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.radians
            moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", poseArray)
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray)
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray)
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds")

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d")
        fieldPub.set(poseArray)

        /* Telemeterize the module states to a Mechanism2d */
        for (i in 0..3) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle)
            moduleDirections[i].setAngle(state.ModuleStates[i].angle)
            moduleSpeeds[i].length = state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed)

            SmartDashboard.putData("Module $i", moduleMechanisms[i])
        }
    }
}
