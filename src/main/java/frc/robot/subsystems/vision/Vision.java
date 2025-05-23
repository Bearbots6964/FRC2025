package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase {

    public static boolean backCamerasEnabled = true;
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    // Pre-allocate lists to reduce object creation during periodic
    private final List<Pose3d> allTagPoses = new ArrayList<>();
    private final List<Pose3d> allRobotPoses = new ArrayList<>();
    private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    private final List<Pose3d> allRobotPosesRejected = new ArrayList<>();
    List<Pose3d> tagPoses = new ArrayList<>();
    List<Pose3d> robotPoses = new ArrayList<>();
    List<Pose3d> robotPosesAccepted = new ArrayList<>();
    List<Pose3d> robotPosesRejected = new ArrayList<>();
    double timer = 0.0;
    double cameraTimer = 0.0;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        System.out.println("│╠╦ Constructing vision!");
        double initializeTime = Timer.getFPGATimestamp();
        System.out.print("│║╠ Assigning I/O interfaces to self... ");
        this.consumer = consumer;
        this.io = io;
        System.out.println("done.");

        // Initialize inputs
        System.out.print("│║╠ Initializing inputs... ");
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            System.out.print(i + 1 + " ");
            inputs[i] = new VisionIOInputsAutoLogged();
        }
        System.out.println("done.");

        // Initialize disconnected alerts
        System.out.print("│║╠ Initializing disconnected alerts... ");
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            System.out.print(i + 1 + " ");
            disconnectedAlerts[i] = new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
        }
        System.out.println("done.");
        System.out.println("│╠ Vision initialized in " + String.format("%.3f", (Timer.getFPGATimestamp() - initializeTime) * 1000.0) + "ms");
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    //  public Rotation2d getTargetX(int cameraIndex) {
    //    return inputs[cameraIndex].latestTargetObservation.tx();
    //  }
    @Override
    public void periodic() {
        timer = Timer.getFPGATimestamp();
        allTagPoses.clear();
        allRobotPoses.clear();
        allRobotPosesAccepted.clear();
        allRobotPosesRejected.clear();

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            cameraTimer = Timer.getFPGATimestamp();
            if (!((cameraIndex == 1 || cameraIndex == 3) && !backCamerasEnabled)) {
                io[cameraIndex].updateInputs(inputs[cameraIndex]);
                Logger.processInputs("Vision/Camera" + cameraIndex, inputs[cameraIndex]);

                disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

                processCameraData(cameraIndex);
            }
            Logger.recordOutput("Vision/Camera" + cameraIndex + "/Loop Time (ms)", (Timer.getFPGATimestamp() - cameraTimer) * 1000.0);
        }

        // Log summary data (less frequently if needed)
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        RobotContainer.getField().getObject("VisionPoses").setPoses(allRobotPoses.stream().map(Pose3d::toPose2d).toList());
        List<Pose2d> targets = new ArrayList<>();
        var curPose = RobotContainer.getField().getRobotPose();
        for (Pose2d tag : allTagPoses.stream().map(Pose3d::toPose2d).toList()) {
            targets.add(curPose);
            targets.add(tag);
        }
        if (targets.size() < 24) {
            for (int i = targets.size(); i < 24; i++) {
                targets.add(curPose);
            }
        }
        RobotContainer.getField().getObject("VisionTags").setPoses(targets);
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Loop Time (ms)", (Timer.getFPGATimestamp() - timer) * 1000.0);
    }

    private void processCameraData(int cameraIndex) {
        tagPoses.clear();
        robotPoses.clear();
        robotPosesRejected.clear();
        robotPosesAccepted.clear();

        // Add tag poses
        for (int tagId : inputs[cameraIndex].tagIds) {
            Optional<Pose3d> tagPose = getAprilTagLayout().getTagPose(tagId);
            tagPose.ifPresent(tagPoses::add);
        }

        // Loop over pose observations
        for (var observation : inputs[cameraIndex].poseObservations) {
            Pose3d pose = observation.pose();

            // Check whether to reject pose
            boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                    || (observation.tagCount() == 1 && observation.ambiguity() > INSTANCE.getMaxAmbiguity())
                    // Cannot be high ambiguity
                    || Math.abs(pose.getZ()) > INSTANCE.getMaxZError()
                    // Must have realistic Z coordinate

                    // Must be within the field boundaries
                    || pose.getX() < 0.0 || pose.getX() > getAprilTagLayout().getFieldLength() || pose.getY() < 0.0 || pose.getY() > getAprilTagLayout().getFieldWidth();

            // Add pose to log
            robotPoses.add(pose);
            if (rejectPose) {
                robotPosesRejected.add(pose);
                continue; // Skip if rejected
            } else {
                robotPosesAccepted.add(pose);
            }

            // Calculate standard deviations
            double averageTagDistance = observation.averageTagDistance();
            int tagCount = observation.tagCount();
            double stdDevFactor = Math.pow(averageTagDistance, 2.0) / (tagCount * 2);
            double linearStdDev = INSTANCE.getLinearStdDevBaseline() * stdDevFactor;
            double angularStdDev = INSTANCE.getAngularStdDevBaseline() * stdDevFactor;

            double[] cameraStdDevFactors = getCameraStdDevFactors();
            if (cameraIndex < cameraStdDevFactors.length) {
                linearStdDev *= cameraStdDevFactors[cameraIndex];
                angularStdDev *= cameraStdDevFactors[cameraIndex];
            }

            // Send vision observation
            consumer.accept(pose.toPose2d(), observation.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }

        // Log camera data
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));

        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
    }

    @FunctionalInterface
    public interface VisionConsumer {

        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
