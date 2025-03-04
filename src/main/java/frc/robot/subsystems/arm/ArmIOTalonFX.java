package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

// TODO: MotionMagic
public class ArmIOTalonFX implements ArmIO {
  protected SparkMax armAxisMotor;
  protected SparkBaseConfig armAxisConfiguration;
  protected SparkClosedLoopController armAxisController;
  protected RelativeEncoder armAxisEncoder;

  public ArmIOTalonFX(SparkBaseConfig armAxisConfiguration) {
    this.armAxisConfiguration = armAxisConfiguration;

    armAxisMotor =
        new SparkMax(
            Constants.ArmConstants.getArmAxisMotorID(), SparkLowLevel.MotorType.kBrushless);
    armAxisController = armAxisMotor.getClosedLoopController();
    armAxisEncoder = armAxisMotor.getEncoder();

    // tryUntilOk(5, () -> armAxisMotor.getConfigurator().apply(armAxisConfiguration, 0.25));
    // tryUntilOk(5, () -> armAxisMotor.setPosition(Units.Degrees.of(0.0), 0.25));

    armAxisConfiguration.smartCurrentLimit(20).idleMode(SparkBaseConfig.IdleMode.kBrake);
    armAxisMotor.configure(
        armAxisConfiguration,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    armAxisController.setReference(0.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAxisAngle = getArmAngleDegrees();
    inputs.armAppliedVolts = armAxisMotor.getAppliedOutput() * armAxisMotor.getBusVoltage();
    inputs.armAppliedCurrentAmps = armAxisMotor.getOutputCurrent();
  }

  @Override
  public void setArmOpenLoop(double output) {
    armAxisMotor.setVoltage(output);
  }

  @Override
  public void setArmAngleDegrees(Angle angle) {
    // armAxisMotor.setControl(motionMagic.withPosition(angle));
    armAxisMotor
        .getClosedLoopController()
        .setReference(angle.in(Units.Rotations), ControlType.kMAXMotionPositionControl);
  }

  @Override
  public Angle getArmAngleDegrees() {
    return Units.Rotations.of(armAxisMotor.getAbsoluteEncoder().getPosition());
  }

  @Override
  public void stopArm() {
    armAxisMotor.set(0.0);
  }
}
