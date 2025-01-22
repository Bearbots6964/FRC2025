/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package frc.robot.subsystems

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*

object VisionSubsystem : SubsystemBase() {
    private val camera = PhotonCamera(Constants.Vision.kCameraName)
    private val photonEstimator = PhotonPoseEstimator(
        Constants.Vision.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.Vision.kRobotToCam
    )

    /**
     * Returns the latest standard deviations of the estimated pose from [ ][.getEstimatedGlobalPose], for use with [ ]. This should
     * only be used when there are targets visible.
     */
    lateinit var estimationStdDevs: Matrix<N3, N1>

    // Simulation
    private var cameraSim: PhotonCameraSim? = null
    private var visionSim: VisionSystemSim? = null

    init {
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)

        // ----- Simulation
        if (RobotBase.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = VisionSystemSim("main")
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim!!.addAprilTags(Constants.Vision.kTagLayout)
            // Create simulated camera properties. These can be set to mimic your actual camera.
            val cameraProp = SimCameraProperties()
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90.0))
            cameraProp.setCalibError(0.35, 0.10)
            cameraProp.fps = 15.0
            cameraProp.avgLatencyMs = 50.0
            cameraProp.latencyStdDevMs = 15.0
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = PhotonCameraSim(camera, cameraProp)
            // Add the simulated camera to view the targets on this simulated field.
            visionSim!!.addCamera(cameraSim, Constants.Vision.kRobotToCam)

            cameraSim!!.enableDrawWireframe(true)
        }
    }

    val estimatedGlobalPose: Optional<EstimatedRobotPose>
        /**
         * The latest estimated robot pose on the field from vision data. This may be empty. This should
         * only be called once per loop.
         *
         *
         * Also includes updates for the standard deviations, which can (optionally) be retrieved with
         * [getEstimationStdDevs]
         *
         * @return An [EstimatedRobotPose] with an estimated pose, estimate timestamp, and targets
         * used for estimation.
         */
        get() {
            var visionEst = Optional.empty<EstimatedRobotPose>()
            for (change in camera.allUnreadResults) {
                visionEst = photonEstimator.update(change)
                updateEstimationStdDevs(visionEst, change.getTargets())

                if (RobotBase.isSimulation()) {
                    visionEst.ifPresentOrElse(
                        { est: EstimatedRobotPose ->
                            simDebugField!!
                                .getObject("VisionEstimation").pose = est.estimatedPose.toPose2d()
                        },
                        {
                            simDebugField!!.getObject("VisionEstimation").setPoses()
                        })
                }
            }
            return visionEst
        }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private fun updateEstimationStdDevs(
        estimatedPose: Optional<EstimatedRobotPose>, targets: List<PhotonTrackedTarget>,
    ) {
        if (estimatedPose.isEmpty) {
            // No pose input. Default to single-tag std devs
            estimationStdDevs = Constants.Vision.kSingleTagStdDevs
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Vision.kSingleTagStdDevs
            var numTags = 0
            var avgDist = 0.0

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (tgt in targets) {
                val tagPose = photonEstimator.fieldTags.getTagPose(tgt.getFiducialId())
                if (tagPose.isEmpty) continue
                numTags++
                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .translation
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().translation)
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                estimationStdDevs = Constants.Vision.kSingleTagStdDevs
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags.toDouble()
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs
                // Increase std devs based on (average) distance
                estStdDevs = if (numTags == 1 && avgDist > 4) VecBuilder.fill(
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                )
                else estStdDevs.times(1 + (avgDist * avgDist / 30))
                estimationStdDevs = estStdDevs
            }
        }
    }

    // ----- Simulation
    fun simulationPeriodic(robotSimPose: Pose2d?) {
        visionSim!!.update(robotSimPose)
    }

    /** Reset pose history of the robot in the vision system simulation.  */
    fun resetSimPose(pose: Pose2d?) {
        if (RobotBase.isSimulation()) visionSim!!.resetRobotPose(pose)
    }

    val simDebugField: Field2d?
        /** A Field2d for visualizing our robot and objects on the field.  */
        get() {
            if (!RobotBase.isSimulation()) return null
            return visionSim!!.debugField
        }
}
