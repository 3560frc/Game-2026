package frc.robot;

import javax.lang.model.type.UnionType;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class PIDMotor {
  private TalonFX motor;
  private ControlRequest request;

  private PIDMotor() {
    motor = null;
  }

  public static PIDMotor init(int motorID, SlotConfigs slotConfigs, MotionMagicConfigs mmconfigs, double gearRatio,
      InvertedValue direction) {
    PIDMotor self = new PIDMotor();

    self.motor = new TalonFX(motorID);
    FeedbackConfigs configs = new FeedbackConfigs();
    configs.SensorToMechanismRatio = gearRatio;
    self.motor.getConfigurator().apply(configs);
    self.motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(direction));

    Slot0Configs slot0Configs = Slot0Configs.from(slotConfigs);
    self.motor.getConfigurator().apply(slot0Configs);

    if (mmconfigs != null) {
      self.motor.getConfigurator().apply(mmconfigs);
    }

    return self;
  }

  public void update() {
    motor.setControl(request);
  }

  private static SlotConfigs createSlotConfigs(double kP, double kI, double kD) {
    SlotConfigs slotConfigs = new SlotConfigs();
    slotConfigs.kP = kP;
    slotConfigs.kI = kI;
    slotConfigs.kD = kD;
    return slotConfigs;
  }

  public static PIDMotor initVelocityOnly(int motorId, double kP, double kI, double kD, InvertedValue direction) {
    return init(motorId, createSlotConfigs(kP, kI, kD), null, 1.0, direction);
  }

  public void setVelocity(double targetRPS) {
    this.request = new VelocityDutyCycle(targetRPS);
  }

  public void set(double targetRotations) {
    MotionMagicVoltage request = new MotionMagicVoltage(targetRotations).withSlot(0);
    this.request = request;
  }
}
