package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class PIDMotor {
  private TalonFX motor;
  private ControlRequest request;
  public boolean disabled;

  private PIDMotor() {
    motor = null;
  }

  public static PIDMotor init(int motorID, SlotConfigs slotConfigs, MotionMagicConfigs mmconfigs, double gearRatio,
      InvertedValue direction) {
    return init(motorID, slotConfigs, mmconfigs, gearRatio, direction, 60);
  }

  public static PIDMotor init(int motorID, SlotConfigs slotConfigs, MotionMagicConfigs mmconfigs, double gearRatio,
      InvertedValue direction, double max_amps) {
    PIDMotor self = new PIDMotor();

    TalonFXConfiguration current_config = new TalonFXConfiguration().withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));

    self.motor = new TalonFX(motorID);
    FeedbackConfigs configs = new FeedbackConfigs();
    configs.SensorToMechanismRatio = gearRatio;
    self.motor.getConfigurator().apply(configs);
    self.motor.getConfigurator().apply(current_config);
    self.motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(direction));

    Slot0Configs slot0Configs = Slot0Configs.from(slotConfigs);
    self.motor.getConfigurator().apply(slot0Configs);

    if (mmconfigs != null) {
      self.motor.getConfigurator().apply(mmconfigs);
    }

    return self;
  }

  public void update() {
    if (!disabled) {
      motor.setControl(request);
    }
  }

  public void setVelocity(double targetRPS) {
    this.request = new MotionMagicVelocityDutyCycle(targetRPS);
  }

  public void set(double targetRotations) {
    MotionMagicVoltage request = new MotionMagicVoltage(targetRotations).withSlot(0);
    this.request = request;
  }

  public void stop() {
    this.request = new VoltageOut(0);
  }
}
