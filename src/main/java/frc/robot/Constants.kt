package frc.robot

import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants {
    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
    }

    object PhysicalProperties {
        object ProgrammingBase {
            private val mass: Mass = Units.Pounds.of(45.0)
            private val momentOfInteria: MomentOfInertia =
                Units.KilogramSquareMeters.of(2.0) // TODO

            object Config {
                private val wheelRadius: Distance = Units.Inches.of(2.0)

                // We are using Kraken X60's. 15.5 ft/s without field-oriented control,
                // and 15.0 ft/s without.
                private val maxVelocity: LinearVelocity = Units.FeetPerSecond.of(15.5)

                private const val coefficentOfFriction = 1.1 // this will need to be changed.
                // just a ballpark estimate for now.
                // very much depends on whether we're on carpet or concrete.

                private val motor: DCMotor =
                    DCMotor.getKrakenX60(1).withReduction(6.75) // per module
                private val currentLimit: Current = Units.Amps.of(40.0)

                private const val motorsPerModule = 1

                val config = ModuleConfig(
                    wheelRadius,
                    maxVelocity,
                    coefficentOfFriction,
                    motor,
                    currentLimit,
                    motorsPerModule
                )
            }

            private val moduleOffsets = arrayOf(
                // front left
                Translation2d(Units.Inches.of(12.125), Units.Inches.of(12.125)),
                // front right
                Translation2d(Units.Inches.of(12.125), Units.Inches.of(-12.125)),
                // back left
                Translation2d(Units.Inches.of(-12.125), Units.Inches.of(12.125)),
                // back right
                Translation2d(Units.Inches.of(-12.125), Units.Inches.of(-12.125)),
            )

            // construct the robot configuration.
            // note the asterisk before the moduleOffsets array -
            // that is the spread operator, allowing us to unwrap the array as a vararg
            val robotConfig = RobotConfig(mass, momentOfInteria, Config.config, *moduleOffsets)
        }

        val activeBase = ProgrammingBase
    }

    object Vision {
        const val kCameraName: String = "YOUR CAMERA NAME"

        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        val kRobotToCam: Transform3d =
            Transform3d(Translation3d(0.5, 0.0, 0.5), Rotation3d(0.0, 0.0, 0.0))

        // The layout of the AprilTags on the field
        val kTagLayout: AprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        val kSingleTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(4.0, 4.0, 8.0)
        val kMultiTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(0.5, 0.5, 1.0)
    }
}



