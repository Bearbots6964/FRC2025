package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void setPosition(Distance position) {
    // gearbox 20:1, sprocket #25 22T (1.889 in radius)
    var inches = position.in(Units.Inches);
    // divide by 2 because the carriage is geared 2:1 relative to the 1st stage
    inches /= 2;
    // divide that by circumference of sprocket to get rotations of gear
    var rotations = inches / (1.889 * Math.PI * 2);
    // multiply by 20 to get rotations of motor
    var motorRotations = rotations * 20;
    io.setPosition(Units.Rotations.of(motorRotations));

  }

  public void setVelocity(LinearVelocity velocity) {
    // gearbox 20:1, sprocket #25 22T (1.889 in radius)
    var inchesPerSecond = velocity.in(Units.InchesPerSecond);
    // divide by 2 because the carriage is geared 2:1 relative to the 1st stage
    inchesPerSecond /= 2;
    // divide that by circumference of sprocket to get rotations of gear
    var rotationsPerSecond = inchesPerSecond / (1.889 * Math.PI * 2);
    // multiply by 20 to get rotations of motor
    var motorRotationsPerSecond = rotationsPerSecond * 20;
    io.setVelocity(Units.RotationsPerSecond.of(motorRotationsPerSecond));
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

}
