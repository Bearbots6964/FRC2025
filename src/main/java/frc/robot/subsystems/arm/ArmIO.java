package frc.robot.subsystems.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public boolean armAxisConnected = false;
        public boolean armFlywheelConnected = false;
        public Angle armAxisAngle = Units.Degree.of(0.0);
        public AngularVelocity armFlywheelAngularVelocity = Units.RadiansPerSecond.of(0.0);
        public Voltage armAppliedVolts = Units.Volts.of(0.0);
        public Current armAppliedCurrent = Units.Amps.of(0.0);
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setArmFlywheelOpenLoop(double output) {}

    default void setArmAxisAngleDegrees(Angle angle) {}

    default Angle getArmAxisAngleDegrees() {
        return Units.Degrees.of(0.0);
    }

    default void setArmFlywheelAngularVelocity(AngularVelocity velocity) {}

    default AngularVelocity getArmFlywheelAngularVelocity() {
        return Units.DegreesPerSecond.of(0.0);
    }

    default void stopArmAxis() {}

    default void stopArmFlywheel() {}
}
