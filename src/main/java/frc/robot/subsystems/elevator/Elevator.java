package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatorConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private SysIdRoutine sysIdRoutine;

    public Elevator(ElevatorIO io) {
        this.io = io;
        if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
            SignalLogger.setPath("/U/sysidlogs/");
            SignalLogger.start();
            sysIdRoutine = new SysIdRoutine(
                    new Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString())),
                    new Mechanism(io::setVoltage, null, this));
        }
    }

    public void setPosition(Distance position) {
        // gearbox 20:1, sprocket #25 22T (1.889 in radius)
        var inches = position.in(Units.Inches);
        // divide by 2 because the carriage is geared 2:1 relative to the 1st stage
        inches /= 2;
        // divide that by circumference of sprocket to get rotations of gear
        var rotations = inches / (1.889 * Math.PI * 2);
        // multiply by 20 to get rotations of motor
        var motorRotations = rotations * 20;
        io.setPosition(Units.Rotations.of(motorRotations));
    }

    public void setRotations(double rotations) {
        io.setPosition(Units.Rotations.of(rotations));
    }

    public void setVelocity(LinearVelocity velocity) {
        // gearbox 20:1, sprocket #25 22T (1.889 in radius)
        var inchesPerSecond = velocity.in(Units.InchesPerSecond);
        // divide by 2 because the carriage is geared 2:1 relative to the 1st stage
        inchesPerSecond /= 2;
        // divide that by circumference of sprocket to get rotations of gear
        var rotationsPerSecond = inchesPerSecond / (1.889 * Math.PI * 2);
        // multiply by 20 to get rotations of motor
        var motorRotationsPerSecond = rotationsPerSecond * 20;
        io.setVelocity(Units.RotationsPerSecond.of(motorRotationsPerSecond));
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Command velocityCommand(DoubleSupplier joystick) {
        return run(() -> io.setOpenLoop(joystick.getAsDouble()));
    }

    public Command sysIdQuasistatic(Direction direction) {
        if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
            return sysIdRoutine.quasistatic(direction);
        } else {
            return null;
        }
    }

    public Command sysIdDynamic(Direction direction) {
        if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
            return sysIdRoutine.dynamic(direction);
        } else {
            return null;
        }
    }

    public Command goToPosition(ElevatorConstants.ElevatorState state) {
        var position = 0.0;
        switch (state) {
            case L1:
                position = ElevatorConstants.L1;
                break;
            case L2:
                position = ElevatorConstants.L2;
                break;
            case L3:
                position = ElevatorConstants.L3;
                break;
            case L4:
                position = ElevatorConstants.L4;
                break;
            case HOME:
            default:
                position = ElevatorConstants.HOME;
                break;
        }
        double finalPosition = position;
        return run(() -> setRotations(finalPosition));
    }
    public Command goToPosition(double position) {
        return run(() -> setRotations(position)).until(() -> Math.abs(inputs.rightMotorPosition.in(Rotations) - position) < 1.0);
    }

    public Command doNothing() {
        return run(io::stop);
    }
}
