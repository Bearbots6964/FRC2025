package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public final class AprilTagPositions {
  public static final HashMap<Integer, Pose2d> WELDED_APRIL_TAG_POSITIONS = new HashMap<>();

  static {
    WELDED_APRIL_TAG_POSITIONS.put(
        1,
        new Pose2d(
            Units.inchesToMeters(657.37),
            Units.inchesToMeters(25.8),
            new Rotation2d(Math.toRadians(126 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        2,
        new Pose2d(
            Units.inchesToMeters(657.37),
            Units.inchesToMeters(291.2),
            new Rotation2d(Math.toRadians(234 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        3,
        new Pose2d(
            Units.inchesToMeters(455.15),
            Units.inchesToMeters(317.15),
            new Rotation2d(Math.toRadians(270 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        4,
        new Pose2d(
            Units.inchesToMeters(365.2),
            Units.inchesToMeters(241.64),
            new Rotation2d(Math.toRadians(180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        5,
        new Pose2d(
            Units.inchesToMeters(365.2),
            Units.inchesToMeters(75.39),
            new Rotation2d(Math.toRadians(180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        6,
        new Pose2d(
            Units.inchesToMeters(530.49),
            Units.inchesToMeters(130.17),
            new Rotation2d(Math.toRadians(300 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        7,
        new Pose2d(
            Units.inchesToMeters(546.87),
            Units.inchesToMeters(158.5),
            new Rotation2d(Math.toRadians(180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        8,
        new Pose2d(
            Units.inchesToMeters(530.49),
            Units.inchesToMeters(186.83),
            new Rotation2d(Math.toRadians(60 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        9,
        new Pose2d(
            Units.inchesToMeters(497.77),
            Units.inchesToMeters(186.83),
            new Rotation2d(Math.toRadians(120 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        10,
        new Pose2d(
            Units.inchesToMeters(481.39),
            Units.inchesToMeters(158.5),
            new Rotation2d(Math.toRadians(180 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        11,
        new Pose2d(
            Units.inchesToMeters(497.77),
            Units.inchesToMeters(130.17),
            new Rotation2d(Math.toRadians(240 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        12,
        new Pose2d(
            Units.inchesToMeters(33.51),
            Units.inchesToMeters(25.8),
            new Rotation2d(Math.toRadians(54 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        13,
        new Pose2d(
            Units.inchesToMeters(33.51),
            Units.inchesToMeters(291.2),
            new Rotation2d(Math.toRadians(306 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        14,
        new Pose2d(
            Units.inchesToMeters(325.68),
            Units.inchesToMeters(241.64),
            new Rotation2d(Math.toRadians(180 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        15,
        new Pose2d(
            Units.inchesToMeters(325.68),
            Units.inchesToMeters(75.39),
            new Rotation2d(Math.toRadians(180 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        16,
        new Pose2d(
            Units.inchesToMeters(235.73),
            Units.inchesToMeters(-0.15),
            new Rotation2d(Math.toRadians(90 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        17,
        new Pose2d(
            Units.inchesToMeters(160.39),
            Units.inchesToMeters(130.17),
            new Rotation2d(Math.toRadians(240 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        18,
        new Pose2d(
            Units.inchesToMeters(144),
            Units.inchesToMeters(158.5),
            new Rotation2d(Math.toRadians(180 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        19,
        new Pose2d(
            Units.inchesToMeters(160.39),
            Units.inchesToMeters(186.83),
            new Rotation2d(Math.toRadians(120 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        20,
        new Pose2d(
            Units.inchesToMeters(193.1),
            Units.inchesToMeters(186.83),
            new Rotation2d(Math.toRadians(60 + 180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        21,
        new Pose2d(
            Units.inchesToMeters(209.49),
            Units.inchesToMeters(158.5),
            new Rotation2d(Math.toRadians(180))));
    WELDED_APRIL_TAG_POSITIONS.put(
        22,
        new Pose2d(
            Units.inchesToMeters(193.1),
            Units.inchesToMeters(130.17),
            new Rotation2d(Math.toRadians(300 + 180))));
  }

  public static final HashMap<Integer, Pose2d> WELDED_RED_CORAL_APRIL_TAG_POSITIONS =
      new HashMap<>();

  static {
    WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(6, WELDED_APRIL_TAG_POSITIONS.get(6));
    WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(7, WELDED_APRIL_TAG_POSITIONS.get(7));
    WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(8, WELDED_APRIL_TAG_POSITIONS.get(8));
    WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(9, WELDED_APRIL_TAG_POSITIONS.get(9));
    WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(10, WELDED_APRIL_TAG_POSITIONS.get(10));
    WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(11, WELDED_APRIL_TAG_POSITIONS.get(11));
  }

  public static final HashMap<Integer, Pose2d> WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS =
      new HashMap<>();

  static {
    WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(17, WELDED_APRIL_TAG_POSITIONS.get(17));
    WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(18, WELDED_APRIL_TAG_POSITIONS.get(18));
    WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(19, WELDED_APRIL_TAG_POSITIONS.get(19));
    WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(20, WELDED_APRIL_TAG_POSITIONS.get(20));
    WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(21, WELDED_APRIL_TAG_POSITIONS.get(21));
    WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(22, WELDED_APRIL_TAG_POSITIONS.get(22));
  }

  public static final HashMap<Integer, Pose2d> ANDYMARK_APRIL_TAG_POSITIONS = new HashMap<>();

  static {
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        1,
        new Pose2d(
            Units.inchesToMeters(656.98),
            Units.inchesToMeters(24.73),
            new Rotation2d(Math.toRadians(126 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        2,
        new Pose2d(
            Units.inchesToMeters(656.98),
            Units.inchesToMeters(291.9),
            new Rotation2d(Math.toRadians(234 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        3,
        new Pose2d(
            Units.inchesToMeters(452.4),
            Units.inchesToMeters(316.21),
            new Rotation2d(Math.toRadians(270 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        4,
        new Pose2d(
            Units.inchesToMeters(365.2),
            Units.inchesToMeters(241.44),
            new Rotation2d(Math.toRadians(180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        5,
        new Pose2d(
            Units.inchesToMeters(365.2),
            Units.inchesToMeters(75.19),
            new Rotation2d(Math.toRadians(180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        6,
        new Pose2d(
            Units.inchesToMeters(530.49),
            Units.inchesToMeters(129.97),
            new Rotation2d(Math.toRadians(300 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        7,
        new Pose2d(
            Units.inchesToMeters(546.87),
            Units.inchesToMeters(158.3),
            new Rotation2d(Math.toRadians(180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        8,
        new Pose2d(
            Units.inchesToMeters(530.49),
            Units.inchesToMeters(186.63),
            new Rotation2d(Math.toRadians(60 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        9,
        new Pose2d(
            Units.inchesToMeters(497.77),
            Units.inchesToMeters(186.63),
            new Rotation2d(Math.toRadians(120 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        10,
        new Pose2d(
            Units.inchesToMeters(481.39),
            Units.inchesToMeters(158.3),
            new Rotation2d(Math.toRadians(180 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        11,
        new Pose2d(
            Units.inchesToMeters(497.77),
            Units.inchesToMeters(129.97),
            new Rotation2d(Math.toRadians(240 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        12,
        new Pose2d(
            Units.inchesToMeters(33.91),
            Units.inchesToMeters(24.73),
            new Rotation2d(Math.toRadians(54 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        13,
        new Pose2d(
            Units.inchesToMeters(33.91),
            Units.inchesToMeters(291.9),
            new Rotation2d(Math.toRadians(306 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        14,
        new Pose2d(
            Units.inchesToMeters(325.68),
            Units.inchesToMeters(241.44),
            new Rotation2d(Math.toRadians(180 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        15,
        new Pose2d(
            Units.inchesToMeters(325.68),
            Units.inchesToMeters(75.19),
            new Rotation2d(Math.toRadians(180 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        16,
        new Pose2d(
            Units.inchesToMeters(238.49),
            Units.inchesToMeters(0.42),
            new Rotation2d(Math.toRadians(90 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        17,
        new Pose2d(
            Units.inchesToMeters(160.39),
            Units.inchesToMeters(129.97),
            new Rotation2d(Math.toRadians(240 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        18,
        new Pose2d(
            Units.inchesToMeters(144),
            Units.inchesToMeters(158.3),
            new Rotation2d(Math.toRadians(180 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        19,
        new Pose2d(
            Units.inchesToMeters(160.39),
            Units.inchesToMeters(186.63),
            new Rotation2d(Math.toRadians(120 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        20,
        new Pose2d(
            Units.inchesToMeters(193.1),
            Units.inchesToMeters(186.63),
            new Rotation2d(Math.toRadians(60 + 180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        21,
        new Pose2d(
            Units.inchesToMeters(209.49),
            Units.inchesToMeters(158.3),
            new Rotation2d(Math.toRadians(180))));
    ANDYMARK_APRIL_TAG_POSITIONS.put(
        22,
        new Pose2d(
            Units.inchesToMeters(193.1),
            Units.inchesToMeters(129.97),
            new Rotation2d(Math.toRadians(300 + 180))));
  }
}
