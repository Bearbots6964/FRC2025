package frc.robot.subsystems.arm;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.automation.superstructure.Position;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class Arm extends SubsystemBase {

  public final Trigger atTargetTrigger;
  private Double targetAngle;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public final LoggedMechanismLigament2d firstSegment;
  private final LoggedMechanismLigament2d secondSegment;
  double timer = 0.0;

  public Arm(ArmIO io, LoggedMechanismLigament2d mechanism) {
    System.out.println("│╠╦ Constructing arm!");
    double initializeTime = Timer.getFPGATimestamp();
    System.out.print("│║╠ Assigning I/O interfaces to self... ");
    this.io = io;
    System.out.println("done.");

    System.out.print("│║╠ Initializing Mechanism2d... ");
    firstSegment =
        mechanism.append(
            new LoggedMechanismLigament2d(
                "Arm First Segment",
                inchesToMeters(13.48),
                23.5,
                6.0,
                new Color8Bit(Color.kOrange)));
    secondSegment =
        firstSegment.append(
            new LoggedMechanismLigament2d(
                "Arm Second Segment",
                inchesToMeters(8.6),
                -60.0,
                6.0,
                new Color8Bit(Color.kDenim)));
    System.out.println("done.");
    System.out.println(
        "│╠╝ Arm initialized in "
            + Robot.Companion.formatTimeDelta(initializeTime)
            + "ms");
    atTargetTrigger = new Trigger(
        () -> Math.abs(inputs.armAxisAngle - targetAngle) < 3.0);
  }

  public void periodic() {
    timer = Timer.getFPGATimestamp();
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    firstSegment.setAngle(inputs.armAxisAngle + 270 + 23.5);

    if (DriverStation.isDisabled()) {
      io.stopArm();
    }
    Logger.recordOutput("Arm/Loop Time (ms)", (Timer.getFPGATimestamp() - timer) * 1000.0);
  }

  public Command stop() {
    return runOnce(io::setGoalToCurrent).andThen(run(io::stopArm)).withName("Arm Stop");
  }

  // TODO: Command Factories?

  public Command moveArm(DoubleSupplier output) {
    return run(() -> io.setArmOpenLoop(output.getAsDouble() * 0.2)).withName("Move Arm");
  }

  public Command moveArmToAngle(Double angle) {
    targetAngle = angle;
    return run(() -> io.setArmAngle(angle))
        .until(atTargetTrigger)
        .withName("Move Arm to Angle");
  }

  public Command moveArmToPosition(Position position) {
    if (position.toState().getArmPosition() != null) {
      targetAngle = position.toState().getArmPosition();
      return run(() -> io.setArmAngle(targetAngle))
          .until(atTargetTrigger)
          .withName("Move Arm to Position");
    } else {
      // nothing
      return Commands.none();
    }
  }

  public Command moveArmToAngleWithoutEnding(Double angle) {
    targetAngle = angle;
    return run(() -> io.setArmAngle(angle)).withName("Move Arm to Angle");
  }

  public Command moveArmAngleDelta(Double delta) {
    targetAngle = targetAngle + delta;
    return runOnce(() -> io.setAngleDelta(delta))
        .until(() -> io.getDistanceFromGoal() < 3.0)
        .withName("Move Arm Delta");
  }

  public double getArmAngle() { // degrees
    return inputs.armAxisAngle;
  }
}
