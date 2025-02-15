package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** The interface to {@link Climber}. */
public interface ClimberIo {
    /** The input to {@link Climber}. */
    @AutoLog
    public class Input {
        public double test;
    }

    /** Update the climber input. */
    public default void update(InputAutoLogged input) {}
}
