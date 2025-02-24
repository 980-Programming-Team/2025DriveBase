package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.Constants;

public class ArmIOSpark implements ArmIO {
  private SparkBase arm;
  private SparkClosedLoopController armPIDController;
  private RelativeEncoder armEncoder;

  private SparkMaxConfig armConfig = new SparkMaxConfig();

  public ArmIOSpark() {
    arm = new SparkMax(Constants.Manipulator.kArm, MotorType.kBrushless);

    configureArm(arm, armConfig);

    armPIDController = arm.getClosedLoopController();
    armEncoder = arm.getEncoder();
  }

  private void configureArm(SparkBase motor, SparkBaseConfig config) {
    config.disableFollowerMode();
    config.inverted(false);
    config.smartCurrentLimit(Constants.Manipulator.Arm.currentLimit);
    config.idleMode(IdleMode.kBrake);

    config.closedLoop.pidf(
        Constants.Manipulator.Arm.kP,
        Constants.Manipulator.Arm.kI,
        Constants.Manipulator.Arm.kD,
        Constants.Manipulator.Arm.kFF);

    config.closedLoop.outputRange(
        Constants.Manipulator.Arm.minOutput, Constants.Manipulator.Arm.maxOutput);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.kArmConnected = (arm.getFirmwareVersion() != 0);
    inputs.armAppliedVoltage = arm.getBusVoltage();
    inputs.armPosMotorRotations = arm.getEncoder().getPosition();
    inputs.armPosAbsMechanismRotations =
        (armEncoder.getPosition() > Constants.Manipulator.Arm.absZeroWrapThreshold)
            ? 0.0
            : (armEncoder.getPosition() / Constants.Manipulator.Arm.motorGearRatio);
    inputs.supplyArmCurrentAmps = arm.getOutputCurrent();
    inputs.armTempCelcius = arm.getMotorTemperature();
  }

  @Override
  public void setArmPosition(double mechanismRotations) {
    double targetPosition = mechanismRotations * Constants.Manipulator.Arm.motorGearRatio;
    armPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  public void seedPivotPosition(double newPositionMechanismRot) {
    armEncoder.setPosition(newPositionMechanismRot * Constants.Manipulator.Arm.motorGearRatio);
  }

  @Override
  public void stop() {
    arm.stopMotor();
  }
}
