package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class WinchIOTalonFX implements WinchIO {
  private final TalonFX winchMotor;
  private final TalonFXConfiguration winchConfig;

  private final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> winchMotorPosition;
  private final StatusSignal<AngularVelocity> winchMotorVelocity;
  private final StatusSignal<Current> winchMotorCurrent;
  private final StatusSignal<Voltage> winchMotorVoltage;

  private final Debouncer winchMotorConnectedDebouncer = new Debouncer(0.5);

  public WinchIOTalonFX(TalonFXConfiguration configuration) {
    System.out.println("│╠╦ Constructing winch I/O!");
    double initializeTime = System.currentTimeMillis();
    System.out.print("│║╠ Assigning configs to self... ");
    this.winchConfig = configuration;
    System.out.println("done.");

    System.out.print("│║╠ Creating mechanism motor... ");
    winchMotor = new TalonFX(Constants.ClimberConstants.getWinchMotorID());
    System.out.println("done.");

    System.out.print("│║╠ Configuring mechanism motor... ");
    tryUntilOk(5, () -> winchMotor.getConfigurator().apply(winchConfig));
    System.out.println("done.");

    System.out.print("│║╠ Configuring status signals... ");
    winchMotorPosition = winchMotor.getPosition();
    winchMotorVelocity = winchMotor.getVelocity();
    winchMotorCurrent = winchMotor.getTorqueCurrent();
    winchMotorVoltage = winchMotor.getMotorVoltage();
    System.out.println("done.");

    System.out.println(
        "│╠╝ Winch I/O initialized in "
            + String.format("%.3f", (System.currentTimeMillis() - initializeTime) * 1000.0)
            + "ms (note: no logging of winch subsystem initialization)");
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    var winchStatus =
        BaseStatusSignal.refreshAll(
            winchMotorPosition, winchMotorVelocity, winchMotorCurrent, winchMotorVoltage);
    inputs.winchConnected = winchMotorConnectedDebouncer.calculate(winchStatus.isOK());
    inputs.winchPositionDegrees = winchMotorPosition.getValue().in(Units.Degrees);
    inputs.winchPositionNominalRotations =
        winchMotorPosition.getValue().in(Units.Rotations) / 60.0;
    inputs.winchVelocityDegreesPerSecond = winchMotorVelocity.getValue().in(Units.DegreesPerSecond);
    inputs.winchAppliedVolts = winchMotorVoltage.getValue().in(Units.Volts);
    inputs.winchAppliedCurrentAmps = winchMotorCurrent.getValue().in(Units.Amps);
  }

  @Override
  public void setWinchOpenLoopVoltage(double output) {
    dutyCycleRequest.withOutput(output).withEnableFOC(true);
    winchMotor.setControl(dutyCycleRequest);
  }

  @Override
  public void setWinchOpenLoop(double output) {
    dutyCycleRequest.withOutput(output).withEnableFOC(true);
    winchMotor.setControl(dutyCycleRequest);
  }

  @Override
  public void setWinchPosition(double position) {
    motionMagicPositionRequest.withPosition(position).withEnableFOC(true);
    winchMotor.setControl(motionMagicPositionRequest);
  }

  @Override
  public void setWinchVelocity(double velocity) {
    motionMagicVelocityRequest.withVelocity(velocity).withEnableFOC(true);
    winchMotor.setControl(motionMagicVelocityRequest);
  }

  @Override
  public double getWinchPosition() {
    return winchMotorPosition.getValue().in(Units.Degrees);
  }

  @Override
  public void stopWinch() {
    winchMotor.set(0);
  }

  @Override
  public void setWinchBrakeMode(NeutralModeValue mode) {
    winchMotor.setNeutralMode(mode);
  }
}
