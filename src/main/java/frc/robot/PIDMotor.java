package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class PIDMotor {
  private TalonFX motor;
  private boolean velocityOnly;

  public PIDMotor() {
    motor = null;
  }

  public static PIDMotor init(int motorID, SlotConfigs slotConfigs, double gearRatio, boolean velocityOnly,
      InvertedValue direction) {
    PIDMotor self = new PIDMotor();
    self.velocityOnly = velocityOnly;

    self.motor = new TalonFX(motorID);
    FeedbackConfigs configs = new FeedbackConfigs();
    configs.SensorToMechanismRatio = gearRatio;
    self.motor.getConfigurator().apply(configs);
    self.motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(direction));

    Slot0Configs slot0Configs = Slot0Configs.from(slotConfigs);
    self.motor.getConfigurator().apply(slot0Configs);

    if (!velocityOnly) {
      MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
      motionMagicConfigs.MotionMagicAcceleration = 1500;
      motionMagicConfigs.MotionMagicCruiseVelocity = 1500;

      self.motor.getConfigurator().apply(motionMagicConfigs);
    }

    return self;
  }

  private SlotConfigs createSlotConfigs(double kP, double kI, double kD) {
    SlotConfigs slotConfigs = new SlotConfigs();
    slotConfigs.kP = kP;
    slotConfigs.kI = kI;
    slotConfigs.kD = kD;
    return slotConfigs;
  }

  public PIDMotor init(int motorId, double kP, double kI, double kD, boolean velocityOnly, InvertedValue direction) {
    return init(motorId, createSlotConfigs(kP, kI, kD), 1.0, velocityOnly, direction);
  }

  public void setVelocity(double targetRPS) {
    motor.setControl(new VelocityDutyCycle(targetRPS));
  }

  public void set(double target) {
    assert (!velocityOnly);
    MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    motor.setControl(request.withPosition(target));
  }
}
