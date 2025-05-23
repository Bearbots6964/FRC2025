package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {

  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
  double timer = 0.0;

  public AlgaeIntake(AlgaeIntakeIO io) {
    double initializeTime = Timer.getFPGATimestamp();
    this.io = io;
    System.out.println(
        "│╠═ Algae Intake initialized in "
            + String.format("%.3f", (Timer.getFPGATimestamp() - initializeTime) * 1000.0)
            + "ms");
  }

  public void periodic() {
    timer = Timer.getFPGATimestamp();
    io.updateInputs(inputs);
    Logger.processInputs("Algae Intake", inputs);
    Logger.recordOutput("Algae Intake/Loop Time (ms)", (Timer.getFPGATimestamp() - timer) * 1000.0);
  }

  public void setArmOpenLoop(double output) {
    io.setArmOpenLoop(output);
  }

  public void setArmPosition(double position) {
    io.setArmPosition(position);
  }

  public void setIntakeOpenLoop(double output) {
    io.setIntakeOpenLoop(output);
  }

  public void setIntakeVelocity(double velocity) {
    io.setIntakeVelocity(velocity);
  }

  public Command runIntake() {
    return run(() -> {
          setIntakeVelocity(Constants.AlgaeIntakeConstants.getIntakeVelocity());
          setArmPosition(Constants.AlgaeIntakeConstants.getArmExtendedPosition());
        })
        .finallyDo(
            () -> {
              setIntakeOpenLoop(0);
              setArmOpenLoop(0);
            });
  }

  public Command stopIntake() {
    return run(() -> setIntakeOpenLoop(0)).alongWith(run(() -> setArmOpenLoop(0)));
  }

  public Command retractIntake() {
    return run(() -> {
          setArmPosition(Constants.AlgaeIntakeConstants.getArmRetractedPosition());
          setIntakeOpenLoop(-0.1);
        })
        .until(
            () ->
                inputs.armMotorPosition
                    < Constants.AlgaeIntakeConstants.getArmRetractedPosition() + 1.5)
        .finallyDo(
            () -> {
              setIntakeOpenLoop(0.0);
              setArmOpenLoop(0.0);
            });
  }
}
