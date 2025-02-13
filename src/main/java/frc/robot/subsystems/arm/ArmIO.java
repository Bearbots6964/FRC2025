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
        public Angle armAngleRad = Units.Degree.of(0.0);
        public AngularVelocity armAngularVelocity = Units.RadiansPerSecond.of(0.0);
        public Voltage armAppliedVolts = Units.Volts.of(0.0);
        public Current armAppliedCurrent = Units.Amps.of(0.0);
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setArmAxisOpenLoop(double output) {}

    default void setArmFlywheelOpenLoop(double output) {}

    default void setArmAxisAngle(Angle angle) {}

    default Angle getArmAxisAngle() {}

    default void setArmFlywheelVelocity(AngularVelocity velocity) {}

    default AngularVelocity getArmFlywheelVelocity() {}

    default void stopArmAxis() {}

    default void stopArmFlywheel() {}
}
