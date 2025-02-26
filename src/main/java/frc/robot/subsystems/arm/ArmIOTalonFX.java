package frc.robot.subsystems.arm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

// TODO: MotionMagic
public class ArmIOTalonFX implements ArmIO {
    protected TalonFXConfiguration ArmFlywheelConfiguration;
    protected TalonFX armFlywheelMotor;

    /* Arm axis TalonFx implementation
    protected TalonFXConfiguration ArmAxisConfiguration;
    protected final TalonFX armAxisMotor;
    */

    protected SparkMax armAxisMotor;
    protected SparkBaseConfig armAxisConfiguration;
    protected SparkClosedLoopController armAxisController;
    protected RelativeEncoder armAxisEncoder;

    protected final MotionMagicVoltage motionMagic;

    public ArmIOTalonFX(SparkBaseConfig armAxisConfiguration, TalonFXConfiguration ArmFlywheelConfiguration) {
        this.armAxisConfiguration = armAxisConfiguration;
        this.ArmFlywheelConfiguration = ArmFlywheelConfiguration;

        armAxisMotor = new SparkMax(Constants.ArmConstants.getArmAxisMotorID(), SparkLowLevel.MotorType.kBrushless);
        armFlywheelMotor = new TalonFX(Constants.ArmConstants.getArmFlywheelMotorID());
        armAxisController = armAxisMotor.getClosedLoopController();
        armAxisEncoder = armAxisMotor.getEncoder();

        // tryUntilOk(5, () -> armAxisMotor.getConfigurator().apply(armAxisConfiguration, 0.25));
        // tryUntilOk(5, () -> armAxisMotor.setPosition(Units.Degrees.of(0.0), 0.25));

        armAxisConfiguration.smartCurrentLimit(50).idleMode(SparkBaseConfig.IdleMode.kBrake);
        armAxisMotor.configure(
                armAxisConfiguration,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        armAxisController.setReference(0.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);

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
        // armAxisMotor.setControl(motionMagic.withPosition(angle));
        armAxisEncoder.setPosition(angle.magnitude());
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
        // return armAxisMotor.getVelocity().getValue();
        return Units.DegreesPerSecond.of(armAxisMotor.get() * Constants.ArmConstants.getMaxAngularVelocity());
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
