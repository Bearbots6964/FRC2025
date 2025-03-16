package frc.robot.subsystems.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public Intake(IntakeIO io) {
        this.io = io;

        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    state -> Logger.recordOutput("Intake/SysIDState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    voltage -> runCharacterization(voltage.in(Units.Volts)), null, this
                )
            );
    }

    public void periodic() {
        io.updateInputs(inputs);

        if (DriverStation.isDisabled()) {
            io.stopIntake();
        }
    }

    public void runCharacterization(double output) {
        io.setIntakeOpenLoopVoltage(output);
    }
}
