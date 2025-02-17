package frc.robot.subsystems.arm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

// TODO: MotionMagic
public class ArmIOTalonFX implements ArmIO {
    protected TalonFXConfiguration ArmAxisConfiguration;
    protected TalonFXConfiguration ArmFlywheelConfiguration;

    protected final TalonFX armAxisMotor;
    protected final TalonFX armFlywheelMotor;

    // protected AngularVelocity

    protected final MotionMagicVoltage motionMagic;

    protected ArmIOTalonFX(TalonFXConfiguration ArmAxisConfiguration, TalonFXConfiguration ArmFlywheelConfiguration) {
        this.ArmAxisConfiguration = ArmAxisConfiguration;
        this.ArmFlywheelConfiguration = ArmFlywheelConfiguration;

        armAxisMotor = new TalonFX(Constants.ArmConstants.getArmAxisMotorID());
        armFlywheelMotor = new TalonFX(Constants.ArmConstants.getArmFlywheelMotorID());

        tryUntilOk(5, () -> armAxisMotor.getConfigurator().apply(ArmAxisConfiguration, 0.25));
        tryUntilOk(5, () -> armAxisMotor.setPosition(Units.Degrees.of(0.0), 0.25));
        tryUntilOk(5, () -> armFlywheelMotor.getConfigurator().apply(ArmFlywheelConfiguration, 0.25));

        motionMagic = new MotionMagicVoltage(0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {}

    @Override
    public void setArmFlywheelOpenLoop(double output) {
        armFlywheelMotor.set(output);
    }

    @Override
    public void setArmAxisAngleDegrees(Angle angle) {
        armAxisMotor.setControl(motionMagic.withPosition(angle));
    }

    @Override
    public Angle getArmAxisAngleDegrees() {
        return Units.Degrees.of(armAxisMotor.get());
    }

    @Override
    public void setArmFlywheelAngularVelocity(AngularVelocity velocity) {
        armFlywheelMotor.set(velocity.magnitude());
    }

    @Override
    public AngularVelocity getArmFlywheelAngularVelocity() {
        return armAxisMotor.getVelocity().getValue();
    }

    @Override
    public void stopArmAxis() {
        armAxisMotor.set(0.0);
    }

    @Override
    public void stopArmFlywheel() {
        armFlywheelMotor.set(0.0);
    }
}
