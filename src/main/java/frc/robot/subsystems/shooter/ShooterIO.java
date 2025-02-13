package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

// The ShooterIO interface just defines the inputs and methods that all implementations use,
// which also allows us to, say, use a log file's inputs instead of a real robot's.
public interface ShooterIO {
    // The ShooterIOInputs class is a container for all the inputs that the shooter subsystem needs.
    // "Inputs" in this case are readings and statistics pulled from the robot's sensors and motors.
    // Basically, if you use it in the code, it's an input.
    @AutoLog
    class ShooterIOInputs {
        public boolean flywheelConnected = false;
        public AngularVelocity flywheelVelocity = Units.RotationsPerSecond.of(0.0);
        public AngularVelocity flywheelTargetVelocity = Units.RotationsPerSecond.of(0.0);
        public Voltage flywheelAppliedVolts = Units.Volts.of(0.0);
        public Current flywheelCurrent = Units.Amps.of(0.0);
    }
    /**
     * Updates the inputs for the shooter subsystem.
     *
     * @param inputs The inputs to update.
     * @implNote This method should be called in your {@code periodic()} method. You'll have to override this method for
     *     your specific IO implementation.
     */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Sets the flywheel motor to open loop control.
     *
     * @param output The output value to set.
     * @implNote There should really only be a few basic methods regarding your motors, or whatever you're controlling.
     *     You want it to be as streamlined between the subsystem and the IO implementation as possible. If you need to
     *     do some really complex thing, you're much better off doing it in the subsystem, and, failing that, the
     *     backend of the implementation. The subsystem and implementation shouldn't really have to communicate that
     *     much.
     */
    default void setShooterOpenLoop(double output) {}

    /**
     * Sets the flywheel motor to a specific velocity.
     *
     * @param velocity The velocity to set.
     * @implNote See the {@link Shooter}'s top level comment. It talks about open loop versus closed loop.
     */
    default void setShooterVelocity(AngularVelocity velocity) {}

    /**
     * Stops the flywheel motor.
     *
     * @implNote It's good practice to have these kinds of safety methods in your code.
     */
    default void stop() {}
}
