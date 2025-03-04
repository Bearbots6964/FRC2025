package frc.robot.subsystems.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public Angle armAxisAngle = Units.Degree.of(0.0);
        public double armAppliedVolts = 0.0;
        public double armAppliedCurrentAmps = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setArmOpenLoop(double output) {}

    default void setArmAngleDegrees(Angle angle) {}

    default Angle getArmAngleDegrees() {
        return Units.Degrees.of(0.0);
    }

    default void stopArm() {}
}
