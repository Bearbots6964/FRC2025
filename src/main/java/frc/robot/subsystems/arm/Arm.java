package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.run;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final Alert armDisconnectedAlert;
    private final SysIdRoutine sysId;

    public Arm(ArmIO io) {
        this.io = io;

        armDisconnectedAlert = new Alert("Disconnected arm motor", Alert.AlertType.kError);

        //TODO: Implement SysId
        //sysId = new SysIdRoutine()
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        }

        armDisconnectedAlert.set(!inputs.armConnected && Constants.getCurrentMode() != Constants.Mode.SIM);
    }

    public void setArmAxisOpenLoop(double output) {
        io.setArmAxisOpenLoop(output);
    }

    public void setArmFlywheelOpenLoop(double output) {
        io.setArmFlywheelOpenLoop(output);
    }

    public void setArmAngleRad(Angle angle) {

    }

    public void setArmVelocityRadPerSec(AngularVelocity velocity) {

    }

    public Angle getArmAngleRad() {

    }

    private void runCharacterization(double output) {
        io.setArmOpenLoop(output);
    }

    // Command Factories

}
