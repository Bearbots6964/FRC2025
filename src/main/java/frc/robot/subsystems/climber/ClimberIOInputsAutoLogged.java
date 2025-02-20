package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("Extended", extended);
        table.put("ClimberAppliedVolts", climberAppliedVolts);
        table.put("ClimberCurrent", climberCurrent);
    }

    @Override
    public void fromLog(LogTable table) {
        extended = table.get("Extended", extended);
        climberAppliedVolts = table.get("ClimberAppliedVolts", climberAppliedVolts);
        climberCurrent = table.get("ClimberCurrent", climberCurrent);
    }

    public ClimberIOInputsAutoLogged clone() {
        ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
        copy.extended = this.extended;
        copy.climberAppliedVolts = this.climberAppliedVolts;
        copy.climberCurrent = this.climberCurrent;
        return copy;
    }
}
