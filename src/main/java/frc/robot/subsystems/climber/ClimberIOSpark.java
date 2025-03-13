package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSpark implements ClimberIO {
  private SparkBase climber;
  private RelativeEncoder encoder;

  private SparkMaxConfig climberConfig;

  public ClimberIOSpark() {
    climber = new SparkMax(Constants.Climber.kClimber, MotorType.kBrushless);

    climberConfig = new SparkMaxConfig();

    configureClimber(climber, climberConfig);
  }

  private void configureClimber(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    motor.clearFaults();
    config.disableFollowerMode();
    config.inverted(false);
    config.smartCurrentLimit(Constants.Climber.currentLimit);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid(3.0, 0.0, 0.0);

    config.closedLoop.outputRange(Constants.Climber.minOutput, Constants.Climber.maxOutput);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.pos = climber.getEncoder().getPosition();
    inputs.supplyCurrentAmps = climber.getOutputCurrent();
    inputs.appliedVoltage = climber.getAppliedOutput();
  }

  @Override
  public void setPosition(double targetPosition) {
    climber.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);

    Logger.recordOutput("Climber/TargetPosition", targetPosition);
  }

  // @Override
  // public void seedPivotPosition(double newPositionMechanismRot) {
  //   armEncoder.setPosition(newPositionMechanismRot * Constants.Manipulator.Arm.motorGearRatio);
  // }

  @Override
  public void stop() {
    climber.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    climberConfig.idleMode(IdleMode.kBrake);
    climber.configure(climberConfig, null, null);
  }
}
