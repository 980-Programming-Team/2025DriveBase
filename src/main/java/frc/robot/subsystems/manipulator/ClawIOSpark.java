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
  private SparkMaxConfig clawConfig;

  // private DigitalInput beamBreak;

  public ClawIOSpark() {
    claw = new SparkMax(Constants.Manipulator.kClaw, MotorType.kBrushless);
    clawConfig = new SparkMaxConfig();

    // beamBreak = new DigitalInput(0);

    configureClaw(claw, clawConfig);
  }

  private void configureClaw(SparkBase motor, SparkBaseConfig config) {
    motor.clearFaults();
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.kClawConnected = (claw.getFirmwareVersion() != 0);
    inputs.clawAppliedVoltage = claw.getBusVoltage();
    inputs.clawSpeedRotationsPerSec = claw.getEncoder().getVelocity();
    inputs.supplyClawCurrentAmps = claw.getOutputCurrent();
    inputs.clawTempCelsius = claw.getMotorTemperature();
    // inputs.kBeamBreak = beamBreak.get();
    // inputs.frontBeamBreakTriggered =
    //     beamBreak.getProximity() < Constants.Arm.proximityDetectionThreshold;
  }

  @Override
  public void setClawSpeed(double speed) {
    claw.set(speed);
  }

  @Override
  public void stop() {
    claw.stopMotor();
  }
}
