package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import frc.robot.constants.Constants;

public class ManipulatorIOSpark implements ManipulatorIO {
  private SparkBase arm;
  private SparkBase claw;
  //private Canandcolor beamBreak;

  private SparkMaxConfig armConfig = new SparkMaxConfig();
  private SparkMaxConfig clawConfig = new SparkMaxConfig();

  public ManipulatorIOSpark() {
    arm = new SparkMax(Constants.Manipulator.kArm, MotorType.kBrushless);
    claw = new SparkMax(Constants.Manipulator.kClaw, MotorType.kBrushless);

    configureArm(arm, armConfig);
    configureClaw(claw, clawConfig);

    //beamBreak = new Canandcolor(Constants.EndEffector.frontBeamBreakID);

  }

  private void configureArm(SparkBase motor, SparkBaseConfig config) {

    config.disableFollowerMode();
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, null, null);
  }

  private void configureClaw(SparkBase motor, SparkBaseConfig config) {

    config.disableFollowerMode();
    config.smartCurrentLimit(Constants.Manipulator.Claw.currentLimit);
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    inputs.armConnected = (arm.getFirmwareVersion() != 0);
    inputs.armAppliedVoltage = arm.getBusVoltage();
    inputs.armSpeedRotationsPerSec = arm.getEncoder().getVelocity();
    inputs.supplyArmCurrentAmps = arm.getOutputCurrent();
    inputs.armTempCelcius = arm.getMotorTemperature();

    inputs.clawConnected = (claw.getFirmwareVersion() != 0);
    inputs.clawAppliedVoltage = claw.getBusVoltage();
    inputs.clawSpeedRotationsPerSec = claw.getEncoder().getVelocity();
    inputs.supplyClawCurrentAmps = claw.getOutputCurrent();
    inputs.clawTempCelcius = claw.getMotorTemperature();
    // inputs.frontBeamBreakTriggered =
    //     beamBreak.getProximity() < Constants.Arm.proximityDetectionThreshold;
  }

  @Override
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  @Override
  public void stop() {
    arm.stopMotor();
  }

}
