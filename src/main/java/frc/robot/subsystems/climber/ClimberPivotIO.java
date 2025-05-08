package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberPivotIO {
  @AutoLog
  public class ClimberPivotIOInputs {
    public double pivotAppliedVolts = 0.0;
    public double pivotAppliedCurrentAmps = 0.0;
    public double pivotVelocityDegreesPerSecond = 0.0;
    public double pivotPositionDegrees = 0.0;
    public double pivotPositionNominalRotations = 0.0;
    public double targetPosition = 0.0;

    public boolean atTarget = false;

    public boolean pivotConnected = false;
  }

  default void updateInputs(ClimberPivotIOInputs inputs) {}

  default void setPivotOpenLoopVoltage(double output) {}

  default void setPivotOpenLoop(double output) {}

  default void setPivotPositionDegrees(double position) {}

  default void setPivotVelocity(double velocity) {}

  default double getPivotPosition() {
    return 0;
  }

  default void stopPivot() {}

  default void setPivotBrakeMode(NeutralModeValue mode) {}
}
