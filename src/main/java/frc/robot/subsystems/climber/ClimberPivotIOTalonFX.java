package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ClimberPivotIOTalonFX implements ClimberPivotIO {
  private final TalonFXS pivotMotor;
  private final TalonFXSConfiguration pivotConfig;

  private final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> pivotMotorPosition;
  private final StatusSignal<AngularVelocity> pivotMotorVelocity;
  private final StatusSignal<Current> pivotMotorCurrent;
  private final StatusSignal<Voltage> pivotMotorVoltage;

  private final Debouncer pivotMotorConnectedDebouncer = new Debouncer(0.5);
  private double targetPosition = 0.0;

  public ClimberPivotIOTalonFX(TalonFXSConfiguration configuration) {
    System.out.println("│╠╦ Constructing climber pivot I/O!");
    double initializeTime = System.currentTimeMillis();
    System.out.print("│║╠ Assigning configs to self... ");
    this.pivotConfig = configuration;
    System.out.println("done.");

    System.out.print("│║╠ Creating mechanism motor... ");
    pivotMotor = new TalonFXS(Constants.ClimberConstants.getPivotMotorID());
    System.out.println("done.");

    System.out.print("│║╠ Configuring mechanism motor... ");
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig));
    System.out.println("done.");

    System.out.print("│║╠ Configuring status signals... ");
    pivotMotorPosition = pivotMotor.getPosition();
    pivotMotorVelocity = pivotMotor.getVelocity();
    pivotMotorCurrent = pivotMotor.getStatorCurrent();
    pivotMotorVoltage = pivotMotor.getMotorVoltage();
    System.out.println("done.");


    System.out.println("│╠╝ Climber pivot I/O initialized in " + String.format("%.3f", (System.currentTimeMillis() - initializeTime) * 1000.0) + "ms (note: no logging of algae intake subsystem initialization)");
  }

  @Override
  public void updateInputs(ClimberPivotIOInputs inputs) {
    var pivotStatus =
        BaseStatusSignal.refreshAll(
            pivotMotorPosition, pivotMotorVelocity, pivotMotorCurrent, pivotMotorVoltage);
    inputs.pivotConnected = pivotMotorConnectedDebouncer.calculate(pivotStatus.isOK());
    inputs.pivotPositionDegrees = pivotMotorPosition.getValue().in(Units.Degrees);
    inputs.pivotPositionNominalRotations = pivotMotorPosition.getValue().in(Units.Rotations) / 100.0;
    inputs.pivotVelocityDegreesPerSecond = pivotMotorVelocity.getValue().in(Units.DegreesPerSecond);
    inputs.pivotAppliedVolts = pivotMotorVoltage.getValue().in(Units.Volts);
    inputs.pivotAppliedCurrentAmps = pivotMotorCurrent.getValue().in(Units.Amps);
    inputs.targetPosition = targetPosition;
    inputs.atTarget = Math.abs(inputs.pivotPositionDegrees - targetPosition) < 7.5;
  }

  @Override
  public void setPivotOpenLoopVoltage(double output) {
    dutyCycleRequest.withOutput(output).withEnableFOC(true);
    pivotMotor.setControl(dutyCycleRequest);
  }

  @Override
  public void setPivotOpenLoop(double output) {
    dutyCycleRequest.withOutput(output).withEnableFOC(true);
    pivotMotor.setControl(dutyCycleRequest);
  }

  @Override
  public void stopPivot() {
    pivotMotor.set(0);
  }

  @Override
  public void setPivotPositionDegrees(double position) {
    targetPosition = position;
    motionMagicPositionRequest.withPosition(Units.Degrees.of(position)).withEnableFOC(true);
    pivotMotor.setControl(motionMagicPositionRequest);
  }

  @Override
  public void setPivotVelocity(double velocity) {
    motionMagicVelocityRequest.withVelocity(velocity).withEnableFOC(true);
    pivotMotor.setControl(motionMagicVelocityRequest);
  }

  @Override
  public double getPivotPosition() {
    return pivotMotorPosition.getValue().in(Units.Degrees);
  }

  @Override
  public void setPivotBrakeMode(NeutralModeValue mode) {
    pivotMotor.setNeutralMode(mode);
  }
}
