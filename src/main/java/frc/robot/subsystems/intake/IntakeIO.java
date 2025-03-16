package frc.robot.subsystems.intake;

import frc.robot.subsystems.elevator.ElevatorIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double intakeFlywheelVoltage = 0.0;
        public double intakeFlywheelCurrent = 0.0;

        public double intakeAxisPosition = 0.0;
        public double intakeAxisVoltage = 0.0;
        public double intakeAxisCurrent = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setIntakeOpenLoopVoltage(double voltage) {}

    default void deployIntake() {}

    default void retractIntake() {}

    default void stopIntake() {}
}
