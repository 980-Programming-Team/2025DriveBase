package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.Constants;

public class ClawIOSpark implements ClawIO {
  private SparkBase claw;
  private SparkMaxConfig clawConfig = new SparkMaxConfig();

  public ClawIOSpark() {
    claw = new SparkMax(Constants.Manipulator.kClaw, MotorType.kBrushless);
    configureClaw(claw, clawConfig);
  }

  private void configureClaw(SparkBase motor, SparkBaseConfig config) {
    config.disableFollowerMode();
    config.inverted(true);
    config.smartCurrentLimit(Constants.Manipulator.Claw.currentLimit);
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.kClawConnected = (claw.getFirmwareVersion() != 0);
    inputs.clawAppliedVoltage = claw.getBusVoltage();
    inputs.clawSpeedRotationsPerSec = claw.getEncoder().getVelocity();
    inputs.supplyClawCurrentAmps = claw.getOutputCurrent();
    inputs.clawTempCelcius = claw.getMotorTemperature();
    // inputs.frontBeamBreakTriggered =
    //     beamBreak.getProximity() < Constants.Arm.proximityDetectionThreshold;
  }

  public void setClawVoltage(double voltage) {
    claw.setVoltage(voltage);
  }

  @Override
  public void stop() {
    claw.stopMotor();
  }
}
