package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Climber extends SubsystemBase {

  private final WinchIO winchIO;
  private final WinchIOInputsAutoLogged winchInputs = new WinchIOInputsAutoLogged();

  private final ClimberPivotIO pivotIO;
  private final ClimberPivotIOInputsAutoLogged pivotInputs = new ClimberPivotIOInputsAutoLogged();
  private final Alert overcurrentAlert = new Alert(
      "Bicep pivot is hitting current limit! You're stuck on something, is the winch engaged?",
      AlertType.kWarning);
  private boolean climbing = false;
  private final LoggedMechanismLigament2d supportMechanism;
  private final LoggedMechanismLigament2d pivot;
  @AutoLogOutput
  private final LoggedMechanism2d mechanism;
  double timer = 0.0;

  public Climber(WinchIO winchIO, ClimberPivotIO pivotIO) {
    this.winchIO = winchIO;
    this.pivotIO = pivotIO;
    mechanism = new LoggedMechanism2d(Units.inchesToMeters(29.5), Units.inchesToMeters(29.5));
    LoggedMechanismRoot2d mechanismRoot2d = mechanism.getRoot("Climber Base",
        Units.inchesToMeters(2.0), Units.inchesToMeters(1.75));
    supportMechanism = mechanismRoot2d.append(
        new LoggedMechanismLigament2d("Climber Support", Units.inchesToMeters(12.5), 90.0, 6.0,
            new Color8Bit(Color.kGray)));
    pivot = supportMechanism.append(
        new LoggedMechanismLigament2d("Climber Pivot", Units.inchesToMeters(14), -50.0, 6.0,
            new Color8Bit(Color.kBlack)));
    pivot.append(
        new LoggedMechanismLigament2d("Intake Finger", Units.inchesToMeters(15.0), -21.6, 2.0,
            new Color8Bit(Color.kSilver)));
  }

  public void periodic() {
    timer = Timer.getFPGATimestamp();
    winchIO.updateInputs(winchInputs);
    Logger.processInputs("Climber Winch", winchInputs);
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Climber Pivot", pivotInputs);

    overcurrentAlert.set(pivotInputs.pivotAppliedCurrentAmps > 35.0);
    pivot.setAngle(-pivotInputs.pivotPositionDegrees);

    if (DriverStation.isDisabled()) {
      winchIO.stopWinch();
      pivotIO.stopPivot();
    }
    Logger.recordOutput("Climber/Loop Time (ms)", (Timer.getFPGATimestamp() - timer) * 1000.0);
  }

  public Command moveClimberOpenLoop(DoubleSupplier winchOutput, DoubleSupplier pivotOutput) {
    return run(() -> {
      winchIO.setWinchOpenLoop(winchOutput.getAsDouble());
      pivotIO.setPivotOpenLoop(pivotOutput.getAsDouble());
    }).withName("Move Climber");
  }

  public Command moveClimberVelocity(DoubleSupplier winchOutput, DoubleSupplier pivotOutput) {
    return run(() -> {
      winchIO.setWinchVelocity(winchOutput.getAsDouble());
      pivotIO.setPivotVelocity(pivotOutput.getAsDouble());
    }).withName("Move Climber");
  }

  public Command moveClimberToIntakePosition() {
    return runOnce(() -> {
      winchIO.setWinchBrakeMode(NeutralModeValue.Coast);
      winchIO.stopWinch();
    }).andThen(run(() -> pivotIO.setPivotPositionDegrees(-21.6)))
        .withName("Move Climber to Intake Position");
  }

  public Command moveClimberToCageCatchPosition() {
    return runOnce(() -> winchIO.setWinchBrakeMode(NeutralModeValue.Coast)).andThen(
            run(() -> pivotIO.setPivotPositionDegrees(ClimberConstants.getPivotCageCatchPosition())))
        .until(() ->
            Math.abs(pivotIO.getPivotPosition() - ClimberConstants.getPivotCageCatchPosition())
                < 6.0).withName("Move Climber to Cage Catch Position");
  }

  public Command moveClimberToCageCatchPositionNoStop() {
    return runOnce(() -> winchIO.setWinchBrakeMode(NeutralModeValue.Coast)).andThen(
            run(() -> pivotIO.setPivotPositionDegrees(ClimberConstants.getPivotCageCatchPosition())))
        .withName("Move Climber to Cage Catch Position");
  }

  public Command climb() {
    return runOnce(() -> {
      pivotIO.setPivotBrakeMode(NeutralModeValue.Brake);
      winchIO.setWinchBrakeMode(NeutralModeValue.Brake);
      pivotIO.stopPivot();
      Elastic.sendNotification(
          new Notification(NotificationLevel.INFO, "Info", "Godspeed, soldier."));
      climbing = true;
    }).andThen(run(() -> winchIO.setWinchOpenLoop(1.0)).until(
            () -> pivotIO.getPivotPosition() > ClimberConstants.getPivotClimbedPosition()))
        .andThen(run(() -> {
          pivotIO.stopPivot();
          pivotIO.setPivotBrakeMode(NeutralModeValue.Brake);
          if (pivotIO.getPivotPosition() < ClimberConstants.getPivotClimbedPosition()) {
            winchIO.setWinchOpenLoop(0.2);
          } else {
            winchIO.stopWinch();
          }
        })).finallyDo(() -> {
          pivotIO.stopPivot();
          pivotIO.setPivotBrakeMode(NeutralModeValue.Brake);
          winchIO.stopWinch();
          climbing = false;
        }).withName("Climb");
  }

  public Command pivotToPosition(double position) {
    return run(() -> pivotIO.setPivotPositionDegrees(position)).until(
        () -> Math.abs(pivotIO.getPivotPosition() - position) < 4.0).withName("Pivot to Position");
  }

  public double getPosition() {
    return pivotIO.getPivotPosition();
  }

  public Command waitForCoral() {
    double a = pivotIO.getPivotPosition();
    return runOnce(pivotIO::stopPivot).andThen(
        Commands.waitUntil(() -> Math.abs(pivotIO.getPivotPosition() - a) > 2.0));
  }
}
