package frc.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

// The ClimberIO interface just defines the inputs and methods that all implementations use,
// which also allows us to, say, use a log file's inputs instead of a real robot's.
public interface ClimberIO {
    // The ClimberIOInputs class is a container for all the inputs that the climber subsystem needs.
    // "Inputs" in this case are readings and statistics pulled from the robot's sensors and motors.
    // Basically, if you use it in the code, it's an input.
    // @AutoLog
    class ClimberIOInputs {
        public boolean extended = false;
        public Voltage climberAppliedVolts = Units.Volts.of(0.0);
        public Current climberCurrent = Units.Amps.of(0.0);
    }
    /**
     * Updates the inputs for the climber subsystem.
     *
     * @param inputs The inputs to update.
     * @implNote This method should be called in your {@code periodic()} method. You'll have to override this method for
     *     your specific IO implementation.
     */
    default void updateInputs(ClimberIOInputs inputs) {}

    default void setExtended(boolean extended) {}
    /**
     * Stops the climber motor
     *
     * @implNote It's good practice to have these kinds of safety methods in your code.
     */
    default void stop() {}
}
