package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.AprilTagPositions
import frc.robot.Constants.PathfindingConstants
import frc.robot.commands.PathfindingFactories.Reef.*
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.vision.Vision
import frc.robot.util.ShortString
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrElse
import kotlin.math.cos
import kotlin.math.sin

object PathfindingFactories {
    fun pathfindToCoralStation(
        drive: Drive, side: CoralStationSide, nudge: Supplier<Translation2d>
    ): Command = drive.followRepulsorField(getSpecificCoralStationPose(side), nudge)

    fun pathfindToCoralStation(
        drive: Drive, side: () -> CoralStationSide, nudge: Supplier<Translation2d>
    ): Command {
        return drive.followRepulsorField({ getSpecificCoralStationPose(side.invoke()) }, nudge)
    }

    fun pathfindToReef(drive: Drive, reef: Reef, nudge: Supplier<Translation2d>): Command {
        val targetPose = getSpecificReefSidePose(reef)
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField(targetPose, nudge))
            .andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReef(drive: Drive, reef: () -> Reef, nudge: Supplier<Translation2d>): Command {
        return Commands.runOnce({ Vision.backCamerasEnabled = false }).andThen(
            drive.followRepulsorField({ getSpecificReefSidePose(reef.invoke()) }, nudge)
        ).andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReefButBackALittle(
        drive: Drive, reef: Reef, nudge: Supplier<Translation2d>
    ): Command {
        val targetPose = getSpecificReefSidePose(reef)
        val truePose = translateCoordinates(
            targetPose, targetPose.rotation.degrees, -Units.inchesToMeters(18.0)
        )
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField(truePose, nudge))
            .andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReefButBackALittle(
        drive: Drive, reef: () -> Reef, nudge: Supplier<Translation2d>
    ): Command {
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField({
                getSpecificReefSidePose(reef.invoke()).let {
                    translateCoordinates(
                        it, it.rotation.degrees, -Units.inchesToMeters(18.0)
                    )
                }
            }, nudge)).andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReefButBackALittleMore(
        drive: Drive, reef: () -> Reef, nudge: Supplier<Translation2d>
    ): Command {
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField({
                getSpecificReefSidePose(reef.invoke()).let {
                    translateCoordinates(
                        it, it.rotation.degrees, -Units.inchesToMeters(22.0)
                    )
                }
            }, nudge)).andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReefButBackALittleLess(
        drive: Drive, reef: () -> Reef, nudge: Supplier<Translation2d>
    ): Command {
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField({
                getSpecificReefSidePose(reef.invoke()).let {
                    translateCoordinates(
                        it, it.rotation.degrees, -Units.inchesToMeters(3.0)
                    )
                }
            }, nudge)).andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }
    fun pathfindToPosition(
        drive: Drive, targetPose: Pose2d, nudge: Supplier<Translation2d>
    ): Command = drive.followRepulsorField(targetPose, nudge)

    // </editor-fold>

    private fun getSpecificCoralStationPose(side: CoralStationSide): Pose2d {
        var multip = 1.0
        val alliance = DriverStation.getAlliance().getOrElse { Alliance.Red }
        val tagPose: Pose2d
        if (alliance == Alliance.Red) {
            if (side == CoralStationSide.LEFT) tagPose = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[1]!!
            else {
                tagPose = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[2]!!
                multip = -1.0
            }
        } else {
            if (side == CoralStationSide.LEFT) tagPose = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[13]!!
            else {
                tagPose = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[12]!!
                multip = -1.0
            }
        }

        return translateCoordinates(
            tagPose!!.transformBy(Transform2d(0.0, 0.0, Rotation2d.fromDegrees(180.0))),
            tagPose.rotation.degrees,
            -PathfindingConstants.finalDistanceFromCoralStationMeters
        ).let {
            translateCoordinates(
                it, it.rotation.degrees + 90, Units.inchesToMeters(17.0)
            )
        }
    }

    private fun getSpecificReefSidePose(reef: Reef): Pose2d {
        val aprilTagsToAlignTo = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS
        val alliance = if (DriverStation.getAlliance().isPresent) DriverStation.getAlliance()
            .get() else Alliance.Red
        val aprilTagNum: Int = when (reef) {
            A, B, AB_ALGAE -> when (alliance) {
                Alliance.Red -> 7
                Alliance.Blue -> 18
            }

            C, D, CD_ALGAE -> when (alliance) {
                Alliance.Red -> 8
                Alliance.Blue -> 17
            }

            E, F, EF_ALGAE -> when (alliance) {
                Alliance.Red -> 9
                Alliance.Blue -> 22
            }

            G, H, GH_ALGAE -> when (alliance) {
                Alliance.Red -> 10
                Alliance.Blue -> 21
            }

            I, J, IJ_ALGAE -> when (alliance) {
                Alliance.Red -> 11
                Alliance.Blue -> 20
            }

            K, L, KL_ALGAE -> when (alliance) {
                Alliance.Red -> 6
                Alliance.Blue -> 19
            }

            else -> when (alliance) { // just use the a and b tags
                Alliance.Red -> 7
                Alliance.Blue -> 18
            }
        }
        val side = when (reef) {
            A, C, E, G, I, K -> ReefSides.LEFT
            B, D, F, H, J, L -> ReefSides.RIGHT
            AB_ALGAE, CD_ALGAE, EF_ALGAE, GH_ALGAE, IJ_ALGAE, KL_ALGAE -> ReefSides.MIDDLE
            else -> ReefSides.RIGHT // idk
        }

        val closestPose = aprilTagsToAlignTo[aprilTagNum]

        val inFrontOfAprilTag: Pose2d = translateCoordinates(
            closestPose!!,
            closestPose.rotation.degrees,
            -PathfindingConstants.finalDistanceFromCoralStationMeters
        )
        val leftOrRightOfAprilTag: Pose2d = when (side) {
            ReefSides.LEFT -> {
                translateCoordinates(
                    inFrontOfAprilTag, closestPose.rotation.degrees + 90,
                    // Offset measured by pit team
                    PathfindingConstants.lateralDistanceFromReefMeters - 0.0508
                )
            }

            ReefSides.RIGHT -> {
                translateCoordinates(
                    inFrontOfAprilTag, closestPose.rotation.degrees + 90,
                    // Offset measured by pit team
                    -PathfindingConstants.lateralDistanceFromReefMeters + 0.0127
                )
            }

            else -> inFrontOfAprilTag
        }


        return leftOrRightOfAprilTag
    }

    enum class CoralStationSide : ShortString {
        LEFT {
            override fun toString() = "Left"
            override fun toShortString(): String = "&L"
        },
        RIGHT {
            override fun toString() = "Right"
            override fun toShortString(): String = "&R"
        },
        NONE {
            override fun toString() = "None"
            override fun toShortString(): String = "&_"
        }
    }

    enum class Reef : ShortString {
        A {
            override fun toString() = "A"
            override fun toShortString(): String = "#A"
        },
        B {
            override fun toString() = "B"
            override fun toShortString(): String = "#B"
        },
        C {
            override fun toString() = "C"
            override fun toShortString(): String = "#C"
        },
        D {
            override fun toString() = "D"
            override fun toShortString(): String = "#D"
        },
        E {
            override fun toString() = "E"
            override fun toShortString(): String = "#E"
        },
        F {
            override fun toString() = "F"
            override fun toShortString(): String = "#F"
        },
        G {
            override fun toString() = "G"
            override fun toShortString(): String = "#G"
        },
        H {
            override fun toString() = "H"
            override fun toShortString(): String = "#H"
        },
        I {
            override fun toString() = "I"
            override fun toShortString(): String = "#I"
        },
        J {
            override fun toString() = "J"
            override fun toShortString(): String = "#J"
        },
        K {
            override fun toString() = "K"
            override fun toShortString(): String = "#K"
        },
        L {
            override fun toString() = "L"
            override fun toShortString(): String = "#L"
        },
        AB_ALGAE {
            override fun toString() = "AB Algae"
            override fun toShortString(): String = "%AB"
        },
        CD_ALGAE {
            override fun toString() = "CD Algae"
            override fun toShortString(): String = "%CD"
        },
        EF_ALGAE {
            override fun toString() = "EF Algae"
            override fun toShortString(): String = "%EF"
        },
        GH_ALGAE {
            override fun toString() = "GH Algae"
            override fun toShortString(): String = "%GH"
        },
        IJ_ALGAE {
            override fun toString() = "IJ Algae"
            override fun toShortString(): String = "%IJ"
        },
        KL_ALGAE {
            override fun toString() = "KL Algae"
            override fun toShortString(): String = "%KL"
        },
        NONE {
            override fun toString() = "None"
            override fun toShortString(): String = "#_"
        }
    }

    enum class ReefSides : ShortString {
        LEFT {
            override fun toString() = "Left"
            override fun toShortString(): String = "?L"
        },
        RIGHT {
            override fun toString() = "Right"
            override fun toShortString(): String = "?R"
        },
        MIDDLE {
            override fun toString() = "Middle"
            override fun toShortString(): String = "?M"
        }
    }

    private fun translateCoordinates(
        originalPose: Pose2d, degreesRotate: Double, distance: Double
    ): Pose2d {
        val newXCoord = originalPose.x + (cos(Math.toRadians(degreesRotate)) * distance)
        val newYCoord = originalPose.y + (sin(Math.toRadians(degreesRotate)) * distance)

        return Pose2d(newXCoord, newYCoord, originalPose.rotation)
    }
}