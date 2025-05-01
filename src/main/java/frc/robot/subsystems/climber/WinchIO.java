package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public class WinchIOInputs {
    public double winchAppliedVolts = 0.0;
    public double winchAppliedCurrentAmps = 0.0;
    public double winchVelocityDegreesPerSecond = 0.0;
    public double winchPositionDegrees = 0.0;
    public double targetPosition = 0.0;

    public boolean atTarget = false;

    public boolean winchConnected = false;
  }

  default void updateInputs(WinchIOInputs inputs) {}

  default void setWinchOpenLoopVoltage(double output) {}

  default void setWinchOpenLoop(double output) {}

  default void setWinchPosition(double position) {}

  default void setWinchVelocity(double velocity) {}

  default double getWinchPosition() {
    return 0;
  }

  default void stopWinch() {}

  default void setWinchBrakeMode(NeutralModeValue mode) {}
}
