package frc.robot.subsystems.arm;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class Arm extends SubsystemBase {

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
            + String.format("%.3f", (Timer.getFPGATimestamp() - initializeTime) * 1000.0)
            + "ms");
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
  public Command setGoalToCurrent() {
    return runOnce(io::setGoalToCurrent);
  }


  // TODO: Command Factories?

  public Command moveArm(DoubleSupplier output) {
    return run(() -> io.setArmOpenLoop(output.getAsDouble() * 0.2)).withName("Move Arm");
  }

  public Command moveArmToAngle(Double angle) {
    return run(() -> io.setArmAngle(angle))
        .until(() -> Math.abs(inputs.armAxisAngle - angle) < 3.0)
        .withName("Move Arm to Angle");
  }

  public Command moveArmToAngleWithoutEnding(Double angle) {
    return run(() -> io.setArmAngle(angle)).withName("Move Arm to Angle");
  }

  public Command moveArmAngleDelta(Double delta) {
    return runOnce(() -> io.setAngleDelta(delta))
        .until(() -> io.getDistanceFromGoal() < 3.0)
        .withName("Move Arm Delta");
  }

  public double getArmAngle() { // degrees
    return inputs.armAxisAngle;
  }
}
