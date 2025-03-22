// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AprilTagPositions;
import frc.robot.Robot;
import frc.robot.subsystems.FieldConstants;
import org.littletonrobotics.junction.Logger;

public final class ReefLocations {
  private ReefLocations() {}

  public static final Pose2d[] BLUE_POSES;
  public static final Pose2d[] RED_POSES;

  public static final Translation2d BLUE_REEF;
  public static final Translation2d RED_REEF;

  static {
    double tag18X = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(18).getX();
    double tag21X = AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(21).getX();
    BLUE_REEF = new Translation2d((tag18X + tag21X) / 2, FieldConstants.fieldWidth / 2);

    var A =
        new Pose2d(
            tag18X - (Units.inchesToMeters(3.5) + Units.inchesToMeters(29.5 / 2)),
            FieldConstants.fieldWidth / 2 + .165,
            Rotation2d.kZero);
    var B =
        new Pose2d(
            tag18X - (Units.inchesToMeters(3.5) + Units.inchesToMeters(29.5 / 2)),
            FieldConstants.fieldWidth / 2 - .165,
            Rotation2d.kZero);

    BLUE_POSES = new Pose2d[12];
    BLUE_POSES[0] = A;
    BLUE_POSES[1] = B;
    for (int i = 2; i < 12; i += 2) {
      var rotAngle = Rotation2d.fromDegrees(30 * i);
      BLUE_POSES[i] = A.rotateAround(BLUE_REEF, rotAngle);
      BLUE_POSES[i + 1] = B.rotateAround(BLUE_REEF, rotAngle);
    }

    RED_REEF =
        BLUE_REEF.rotateAround(
            new Translation2d(FieldConstants.fieldLength / 2, FieldConstants.fieldWidth / 2),
            Rotation2d.kPi);
    RED_POSES = new Pose2d[12];
    for (int i = 0; i < 12; i++) {
      RED_POSES[i] =
          BLUE_POSES[i].rotateAround(
              new Translation2d(FieldConstants.fieldLength / 2, FieldConstants.fieldWidth / 2),
              Rotation2d.kPi);
    }
  }

  public enum ReefBranch {
    A(0),
    B(1),
    C(2),
    D(3),
    E(4),
    F(5),
    G(6),
    H(7),
    I(8),
    J(9),
    K(10),
    L(11);

    final int id;

    ReefBranch(int id) {
      this.id = id;
    }
  }

  public static Pose2d getScoringLocation(ReefBranch reefBranch) {
    return (Robot.getAlliance().equals(Alliance.Red) ? RED_POSES : BLUE_POSES)[reefBranch.id];
  }

  public static void log() {
    Logger.recordOutput("Reef Scoring Locations/Blue", BLUE_POSES);
    Logger.recordOutput("Reef Scoring Locations/Red", RED_POSES);
  }
}
