package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public boolean leftMotorConnected = false;
    public Temperature leftMotorTemperature = Units.Celsius.of(0.0);
    public Angle leftMotorPosition = Units.Degrees.of(0.0);
    public AngularVelocity leftMotorVelocity = Units.DegreesPerSecond.of(0.0);
    public Voltage leftMotorVoltage = Units.Volts.of(0.0);
    public Current leftMotorCurrent = Units.Amps.of(0.0);

    public boolean rightMotorConnected = false;
    public Temperature rightMotorTemperature = Units.Celsius.of(0.0);
    public Angle rightMotorPosition = Units.Degrees.of(0.0);
    public AngularVelocity rightMotorVelocity = Units.DegreesPerSecond.of(0.0);
    public Voltage rightMotorVoltage = Units.Volts.of(0.0);
    public Current rightMotorCurrent = Units.Amps.of(0.0);

    public AngularVelocity targetVelocity = Units.RotationsPerSecond.of(0.0);
    public Angle targetPosition = Units.Degrees.of(0.0);
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void setPosition(Angle position) {}

  default void stop() {}

}
