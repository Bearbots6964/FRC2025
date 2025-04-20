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
        var currentSide = side.invoke()
        var targetPose = getSpecificCoralStationPose(side.invoke())
        return drive.followRepulsorField(targetPose, nudge).deadlineFor(Commands.run({
            if (currentSide != side.invoke()) {
                targetPose = getSpecificCoralStationPose(side.invoke()); currentSide = side.invoke()
            }
        }))
    }

    fun pathfindToReef(drive: Drive, reef: Reef, nudge: Supplier<Translation2d>): Command {
        val targetPose = getSpecificReefSidePose(reef)
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField(targetPose, nudge))
            .andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReef(drive: Drive, reef: () -> Reef, nudge: Supplier<Translation2d>): Command {
        var currentReef = reef.invoke()
        var targetPose = getSpecificReefSidePose(reef.invoke())
        return Commands.runOnce({ Vision.backCamerasEnabled = false }).andThen(
            drive.followRepulsorField(targetPose, nudge).deadlineFor(Commands.run({
                if (currentReef != reef.invoke()) {
                    targetPose = getSpecificReefSidePose(reef.invoke()); currentReef = reef.invoke()
                }
            }))
        ).andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReefButBackALittle(
        drive: Drive, reef: Reef, nudge: Supplier<Translation2d>
    ): Command {
        val targetPose = getSpecificReefSidePose(reef)
        val truePose = translateCoordinates(
            targetPose, targetPose.rotation.degrees, -Units.inchesToMeters(16.0)
        )
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField(truePose, nudge))
            .andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToReefButBackALittle(
        drive: Drive, reef: () -> Reef, nudge: Supplier<Translation2d>
    ): Command {
        var currentReef = reef.invoke()
        var targetPose = getSpecificReefSidePose(reef.invoke()).let {
            translateCoordinates(
                it, it.rotation.degrees, -Units.inchesToMeters(16.0)
            )
        }
        return Commands.runOnce({ Vision.backCamerasEnabled = false })
            .andThen(drive.followRepulsorField(targetPose, nudge).deadlineFor(Commands.run({
                if (currentReef != reef.invoke()) {
                    targetPose = getSpecificReefSidePose(reef.invoke()).let {
                        translateCoordinates(
                            it, it.rotation.degrees, -Units.inchesToMeters(16.0)
                        )
                    }; currentReef = reef.invoke()
                }
            }))).andThen(Commands.runOnce({ Vision.backCamerasEnabled = true }))
    }

    fun pathfindToPosition(
        drive: Drive, targetPose: Pose2d, nudge: Supplier<Translation2d>
    ): Command = drive.followRepulsorField(targetPose, nudge)

    // </editor-fold>

    private fun getSpecificCoralStationPose(side: CoralStationSide): Pose2d {
        val alliance = DriverStation.getAlliance().getOrElse { Alliance.Red }
        val tagPose = if (alliance == Alliance.Red) {
            if (side == CoralStationSide.LEFT) AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[1]
            else AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[2]
        } else {
            if (side == CoralStationSide.LEFT) AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[13]
            else AprilTagPositions.WELDED_APRIL_TAG_POSITIONS[12]
        }

        return translateCoordinates(
            tagPose!!.transformBy(Transform2d(0.0, 0.0, Rotation2d.fromDegrees(180.0))),
            tagPose.rotation.degrees,
            -PathfindingConstants.finalDistanceFromCoralStationMeters
        ).let {
            translateCoordinates(
                it, it.rotation.degrees + 90, 0.06985
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
        }
        val side = when (reef) {
            A, C, E, G, I, K -> ReefSides.LEFT
            B, D, F, H, J, L -> ReefSides.RIGHT
            AB_ALGAE, CD_ALGAE, EF_ALGAE, GH_ALGAE, IJ_ALGAE, KL_ALGAE -> ReefSides.MIDDLE
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

    enum class CoralStationSide {
        LEFT {
            override fun toString() = "Left"
        },
        RIGHT {
            override fun toString() = "Right"
        }
    }

    enum class Reef {
        A {
            override fun toString() = "A"
        },
        B {
            override fun toString() = "B"
        },
        C {
            override fun toString() = "C"
        },
        D {
            override fun toString() = "D"
        },
        E {
            override fun toString() = "E"
        },
        F {
            override fun toString() = "F"
        },
        G {
            override fun toString() = "G"
        },
        H {
            override fun toString() = "H"
        },
        I {
            override fun toString() = "I"
        },
        J {
            override fun toString() = "J"
        },
        K {
            override fun toString() = "K"
        },
        L {
            override fun toString() = "L"
        },
        AB_ALGAE {
            override fun toString() = "AB Algae"
        },
        CD_ALGAE {
            override fun toString() = "CD Algae"
        },
        EF_ALGAE {
            override fun toString() = "EF Algae"
        },
        GH_ALGAE {
            override fun toString() = "GH Algae"
        },
        IJ_ALGAE {
            override fun toString() = "IJ Algae"
        },
        KL_ALGAE {
            override fun toString() = "KL Algae"
        }
    }

    enum class ReefSides {
        LEFT {
            override fun toString() = "Left"
        },
        RIGHT {
            override fun toString() = "Right"
        },
        MIDDLE {
            override fun toString() = "Middle"
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