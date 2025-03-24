package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final WinchIO winchIO;
  private final WinchIOInputsAutoLogged winchInputs = new WinchIOInputsAutoLogged();

  private final ClimberPivotIO pivotIO;
  private final ClimberPivotIOInputsAutoLogged pivotInputs = new ClimberPivotIOInputsAutoLogged();

  public Climber(WinchIO winchIO, ClimberPivotIO pivotIO) {
    this.winchIO = winchIO;
    this.pivotIO = pivotIO;
  }

  public void periodic() {
    winchIO.updateInputs(winchInputs);
    Logger.processInputs("Climber Winch", winchInputs);
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Climber Pivot", pivotInputs);

    if (DriverStation.isDisabled()) {
      winchIO.stopWinch();
      pivotIO.stopPivot();
    }
  }

  public Command moveClimberOpenLoop(DoubleSupplier winchOutput, DoubleSupplier pivotOutput) {
    return run(() -> {
          winchIO.setWinchOpenLoop(winchOutput.getAsDouble());
          pivotIO.setPivotOpenLoop(pivotOutput.getAsDouble());
        })
        .withName("Move Climber");
  }

  public Command moveClimberVelocity(DoubleSupplier winchOutput, DoubleSupplier pivotOutput) {
    return run(() -> {
          winchIO.setWinchVelocity(winchOutput.getAsDouble());
          pivotIO.setPivotVelocity(pivotOutput.getAsDouble());
        })
        .withName("Move Climber");
  }

  public Command moveClimberToIntakePosition() {
    return runOnce(
            () -> {
              winchIO.setWinchBrakeMode(NeutralModeValue.Coast);
              winchIO.stopWinch();
            })
        .andThen(run(() -> pivotIO.setPivotPositionDegrees(-21.6)))
        .withName("Move Climber to Intake Position");
  }

  public Command moveClimberToCageCatchPosition() {
    return runOnce(() -> winchIO.setWinchBrakeMode(NeutralModeValue.Coast))
        .andThen(run(() -> pivotIO.setPivotPositionDegrees(ClimberConstants.getPivotCageCatchPosition()))).until(() -> Math.abs(pivotIO.getPivotPosition() - ClimberConstants.getPivotCageCatchPosition()) < 6.0)
        .withName("Move Climber to Cage Catch Position");
  }

  public Command moveClimberToCageCatchPositionNoStop() {
    return runOnce(() -> winchIO.setWinchBrakeMode(NeutralModeValue.Coast))
        .andThen(run(() -> pivotIO.setPivotPositionDegrees(ClimberConstants.getPivotCageCatchPosition())))
        .withName("Move Climber to Cage Catch Position");
  }

  public Command climb() {
    return runOnce(
            () -> {
              pivotIO.setPivotBrakeMode(NeutralModeValue.Brake);
              winchIO.setWinchBrakeMode(NeutralModeValue.Brake);
              pivotIO.stopPivot();
              Elastic.sendNotification(
                  new Notification(NotificationLevel.INFO, "Info", "Godspeed, soldier."));
            })
        .andThen(
            run(() -> winchIO.setWinchOpenLoop(0.60))
                .until(
                    () -> pivotIO.getPivotPosition() > ClimberConstants.getPivotClimbedPosition()))
        .andThen(
            run(
                () -> {
                  pivotIO.stopPivot();
                  pivotIO.setPivotBrakeMode(NeutralModeValue.Brake);
                  winchIO.stopWinch();
                }))
        .finallyDo(
            () -> {
              pivotIO.stopPivot();
              pivotIO.setPivotBrakeMode(NeutralModeValue.Brake);
              winchIO.stopWinch();
            })
        .withName("Climb");
  }

  public Command pivotToPosition(double position) {
    return run(() -> pivotIO.setPivotPositionDegrees(position))
        .until(() -> Math.abs(pivotIO.getPivotPosition() - position) < 4.0)
        .withName("Pivot to Position");
  }

  public double getPosition() {
    return pivotIO.getPivotPosition();
  }
}
