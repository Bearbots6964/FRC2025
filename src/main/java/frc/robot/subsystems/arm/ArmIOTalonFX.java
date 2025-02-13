package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

//TODO: MotionMagic
public class ArmIOTalonFX implements ArmIO{
    protected TalonFXConfiguration ArmAxisConfiguration;
    protected TalonFXConfiguration ArmFlywheelConfiguration;

    protected final TalonFX armAxisMotor;
    protected final TalonFX armFlywheelMotor;

    protected AngularVelocity

    protected ArmIOTalonFX(TalonFXConfiguration ArmAxisConfiguration, TalonFXConfiguration ArmFlywheelConfiguration) {
        this.ArmAxisConfiguration = ArmAxisConfiguration;
        this.ArmFlywheelConfiguration = ArmFlywheelConfiguration;

        armAxisMotor = new TalonFX(Constants.ArmConstants.armAxisMotorID);
        armFlywheelMotor = new TalonFX(Constants.ArmConstants.armFlywheelMotorID);

        tryUntilOk(5, () -> armAxisMotor.getConfigurator().apply(ArmAxisConfiguration, 0.25));
        tryUntilOk(5, () -> armAxisMotor.setPosition(Units.Degrees.of(0.0), 0.25));
        tryUntilOk(5, () -> armFlywheelMotor.getConfigurator().apply(ArmFlywheelConfiguration, 0.25));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

    }

    @Override
    public void setArmAxisOpenLoop(double output) {

    }

    @Override
    public void setArmFlywheelOpenLoop(double output) {
        armFlywheelMotor.set(output);
    }

    @Override
    public void setArmAxisAngle(Angle angle) {

    }

    @Override
    public Angle getArmAxisAngle() {
        return Units.Degrees.of(armAxisMotor.get());
    }

    @Override
    public void setArmFlywheelVelocity(AngularVelocity velocity) {

    }

    @Override
    public AngularVelocity getArmFlywheelVelocity() {
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
