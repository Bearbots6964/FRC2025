package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import lombok.Setter;

/**
 * The RepulsorFieldPlanner class is responsible for calculating the forces exerted by various
 * obstacles on the field and determining the resulting movement vectors for a robot.
 */
public class RepulsorFieldPlanner {

  /** Abstract class representing an obstacle with a certain strength and polarity. */
  private abstract static class Obstacle {
    double strength;
    boolean positive;

    /**
     * Constructs an Obstacle.
     *
     * @param strength the strength of the obstacle
     * @param positive whether the obstacle is repulsive (true) or attractive (false)
     */
    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    /**
     * Calculates the force exerted by the obstacle at a given position.
     *
     * @param position the current position
     * @param target the target position
     * @return the force exerted by the obstacle
     */
    public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d target);

    /**
     * Converts a distance to a force magnitude based on the obstacle's strength and range.
     *
     * @param dist the distance from the obstacle
     * @param maxRange the maximum effective range of the obstacle
     * @return the calculated force magnitude
     */
    protected double distToForceMag(double dist, double maxRange) {
      if (Math.abs(dist) > maxRange) {
        return 0;
      }
      if (MathUtil.isNear(0, dist, 1e-2)) {
        dist = 1e-2;
      }
      var forceMag = strength / (dist * dist);
      forceMag -= strength / (maxRange * maxRange);
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }
  }

  /** Represents a teardrop-shaped obstacle with a primary and tail region. */
  private static class TeardropObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double primaryRadius;
    final double tailStrength;
    final double tailLength;

    /**
     * Constructs a TeardropObstacle.
     *
     * @param loc the location of the obstacle
     * @param primaryStrength the strength of the primary region
     * @param primaryMaxRange the maximum range of the primary region
     * @param primaryRadius the radius of the primary region
     * @param tailStrength the strength of the tail region
     * @param tailLength the length of the tail region
     */
    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailLength = tailLength + primaryMaxRange;
    }

    /**
     * Calculates the force exerted by the teardrop obstacle at a given position.
     *
     * @param position the current position
     * @param target the target position
     * @return the force exerted by the obstacle
     */
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysPoint = new Translation2d(tailLength, targetToLoc.getAngle()).plus(loc);

      var positionToLocation = position.minus(loc);
      var positionToLocationDistance = positionToLocation.getNorm();
      Translation2d outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        outwardsForce =
            new Translation2d(
                distToForceMag(
                    Math.max(positionToLocationDistance - primaryRadius, 0),
                    primaryMaxRange - primaryRadius),
                positionToLocation.getAngle());
      } else {
        outwardsForce = Translation2d.kZero;
      }

      var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
      var distanceAlongLine = positionToLine.getX();

      Translation2d sidewaysForce;
      var distanceScalar = distanceAlongLine / tailLength;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        var secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        var distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          double strength;
          if (distanceAlongLine < primaryMaxRange) {
            strength = tailStrength * (distanceAlongLine / primaryMaxRange);
          } else {
            strength =
                -tailStrength * distanceAlongLine / (tailLength - primaryMaxRange)
                    + tailLength * tailStrength / (tailLength - primaryMaxRange);
          }
          strength *= 1 - distanceToLine / secondaryMaxRange;

          var sidewaysMag = tailStrength * strength * (secondaryMaxRange - distanceToLine);
          // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
          var sidewaysTheta =
              target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
          sidewaysForce =
              new Translation2d(
                  sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                  targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Translation2d.kZero;
        }
      } else {
        sidewaysForce = Translation2d.kZero;
      }

      return outwardsForce.plus(sidewaysForce);
    }
  }

  /** Represents a horizontal obstacle. */
  static class HorizontalObstacle extends Obstacle {
    final double y;
    final double maxRange;

    /**
     * Constructs a HorizontalObstacle.
     *
     * @param y the y-coordinate of the obstacle
     * @param strength the strength of the obstacle
     * @param maxRange the maximum effective range of the obstacle
     * @param positive whether the obstacle is repulsive (true) or attractive (false)
     */
    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    /**
     * Calculates the force exerted by the horizontal obstacle at a given position.
     *
     * @param position the current position
     * @param target the target position
     * @return the force exerted by the obstacle
     */
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getY() - y);
      if (dist > maxRange) {
        return Translation2d.kZero;
      }
      return new Translation2d(0, distToForceMag(y - position.getY(), maxRange));
    }
  }

  /** Represents a vertical obstacle. */
  private static class VerticalObstacle extends Obstacle {
    final double x;
    final double maxRange;

    /**
     * Constructs a VerticalObstacle.
     *
     * @param x the x-coordinate of the obstacle
     * @param strength the strength of the obstacle
     * @param maxRange the maximum effective range of the obstacle
     * @param positive whether the obstacle is repulsive (true) or attractive (false)
     */
    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    /**
     * Calculates the force exerted by the vertical obstacle at a given position.
     *
     * @param position the current position
     * @param target the target position
     * @return the force exerted by the obstacle
     */
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getX() - x);
      if (dist > maxRange) {
        return Translation2d.kZero;
      }
      return new Translation2d(distToForceMag(x - position.getX(), maxRange), 0);
    }
  }

  /** Represents a line obstacle. */
  private static class LineObstacle extends Obstacle {
    final Translation2d startPoint;
    final Translation2d endPoint;
    final double length;
    final Rotation2d angle;
    final Rotation2d inverseAngle;
    final double maxRange;

    /**
     * Constructs a LineObstacle.
     *
     * @param start the start point of the line
     * @param end the end point of the line
     * @param strength the strength of the obstacle
     * @param maxRange the maximum effective range of the obstacle
     */
    public LineObstacle(Translation2d start, Translation2d end, double strength, double maxRange) {
      super(strength, true);
      startPoint = start;
      endPoint = end;
      var delta = end.minus(start);
      length = delta.getNorm();
      angle = delta.getAngle();
      inverseAngle = angle.unaryMinus();
      this.maxRange = maxRange;
    }

    /**
     * Calculates the force exerted by the line obstacle at a given position.
     *
     * @param position the current position
     * @param target the target position
     * @return the force exerted by the obstacle
     */
    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var positionToLine = position.minus(startPoint).rotateBy(inverseAngle);
      if (positionToLine.getX() > 0 && positionToLine.getX() < length) {
        return new Translation2d(
            Math.copySign(distToForceMag(positionToLine.getY(), maxRange), positionToLine.getY()),
            angle.rotateBy(Rotation2d.kCCW_90deg));
      }
      Translation2d closerPoint;
      if (positionToLine.getX() <= 0) {
        closerPoint = startPoint;
      } else {
        closerPoint = endPoint;
      }
      return new Translation2d(
          distToForceMag(position.getDistance(closerPoint), maxRange),
          position.minus(closerPoint).getAngle());
    }
  }

  static final double SOURCE_X = 1.75;
  static final double SOURCE_Y = 1.25;
  static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          // Reef
          new TeardropObstacle(ReefLocations.BLUE_REEF, 1, 2.5, .83, 3, 2),
          new TeardropObstacle(ReefLocations.RED_REEF, 1, 2.5, .83, 3, 2),
          // Walls
          new HorizontalObstacle(0.0, 0.5, .5, true),
          new HorizontalObstacle(Constants.FIELD_WIDTH_METERS, 0.5, .5, false),
          new VerticalObstacle(0.0, 0.5, .5, true),
          new VerticalObstacle(Constants.FIELD_LENGTH_METERS, 0.5, .5, false),
          // Sources
          new LineObstacle(new Translation2d(0, SOURCE_Y), new Translation2d(SOURCE_X, 0), .5, .5),
          new LineObstacle(
              new Translation2d(0, Constants.FIELD_WIDTH_METERS - SOURCE_Y),
              new Translation2d(SOURCE_X, Constants.FIELD_WIDTH_METERS),
              .5,
              .5),
          new LineObstacle(
              new Translation2d(Constants.FIELD_LENGTH_METERS, SOURCE_Y),
              new Translation2d(Constants.FIELD_LENGTH_METERS - SOURCE_X, 0),
              .5,
              .5),
          new LineObstacle(
              new Translation2d(
                  Constants.FIELD_LENGTH_METERS, Constants.FIELD_WIDTH_METERS - SOURCE_Y),
              new Translation2d(
                  Constants.FIELD_LENGTH_METERS - SOURCE_X, Constants.FIELD_WIDTH_METERS),
              .5,
              .5));

  /**
   * -- SETTER --
   *  Sets the goal position.
   *
   * @param goal the new goal position
   */
  @Setter
  private Translation2d goal = new Translation2d(1, 1);

  private static final int ARROWS_X = 40;
  private static final int ARROWS_Y = 20;

  private Translation2d lastGoal;
  private Pose2d[] arrowList;

  /**
   * Returns a grid of arrows representing the forces at various points on the field.
   *
   * @return an array of Pose2d objects representing the arrows
   */
  public Pose2d[] getArrows() {
    if (goal.equals(lastGoal)) {
      return arrowList;
    }
    var list = new ArrayList<Pose2d>();
    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(
                x * Constants.FIELD_LENGTH_METERS / ARROWS_X,
                y * Constants.FIELD_WIDTH_METERS / ARROWS_Y);
        var force = getObstacleForce(translation, goal);
        if (force.getNorm() > 1e-6) {
          var rotation = force.getAngle();

          list.add(new Pose2d(translation, rotation));
        }
      }
    }
    lastGoal = goal;
    arrowList = list.toArray(new Pose2d[0]);
    return arrowList;
  }

  /**
   * Calculates the force exerted by the goal at a given position.
   *
   * @param curLocation the current position
   * @param goal the goal position
   * @return the force exerted by the goal
   */
  Translation2d getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Translation2d();
    }
    var direction = displacement.getAngle();
    var mag = (1 + 1.0 / (1e-6 + displacement.getNorm()));
    return new Translation2d(mag, direction);
  }

  /**
   * Calculates the total force exerted by all obstacles at a given position.
   *
   * @param curLocation the current position
   * @param target the target position
   * @return the total force exerted by all obstacles
   */
  Translation2d getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Translation2d.kZero;
    for (Obstacle obs : FIELD_OBSTACLES) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  /**
   * Calculates the total force exerted by the goal and all obstacles at a given position.
   *
   * @param curLocation the current position
   * @param target the target position
   * @return the total force exerted by the goal and all obstacles
   */
  Translation2d getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
  }

  /** Represents a sample of the repulsor field at a given position. */
  public record RepulsorSample(Translation2d intermediateGoal, double vx, double vy) {}

  /**
   * Samples the repulsor field at a given position and calculates the resulting movement vector.
   *
   * @param curTrans the current position
   * @param maxSpeed the maximum speed
   * @param slowdownDistance the distance at which to start slowing down
   * @return a RepulsorSample representing the resulting movement vector
   */
  public RepulsorSample sampleField(
      Translation2d curTrans, double maxSpeed, double slowdownDistance) {
    var err = curTrans.minus(goal);
    var netForce = getForce(curTrans, goal);

    double stepSize_m;
    if (err.getNorm() < slowdownDistance) {
      stepSize_m = MathUtil.interpolate(0, maxSpeed * 0.02, err.getNorm() / slowdownDistance);
    } else {
      stepSize_m = maxSpeed * 0.02;
    }
    var step = new Translation2d(stepSize_m, netForce.getAngle());
    return new RepulsorSample(curTrans.plus(step), step.getX() / 0.02, step.getY() / 0.02);
  }
}
