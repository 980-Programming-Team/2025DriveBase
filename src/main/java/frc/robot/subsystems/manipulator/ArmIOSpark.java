package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSpark implements ArmIO {
  private SparkBase arm;
  private RelativeEncoder encoder;

  private SparkMaxConfig armConfig;

  public ArmIOSpark() {
    arm = new SparkMax(Constants.Manipulator.kArm, MotorType.kBrushless);

    armConfig = new SparkMaxConfig();

    configureArm(arm, armConfig);
  }

  private void configureArm(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    motor.clearFaults();
    config.disableFollowerMode();
    config.inverted(false);
    config.smartCurrentLimit(Constants.Manipulator.Arm.currentLimit);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid(
        Constants.Manipulator.Arm.kP, Constants.Manipulator.Arm.kI, Constants.Manipulator.Arm.kD);

    config.closedLoop.outputRange(
        Constants.Manipulator.Arm.minOutput, Constants.Manipulator.Arm.maxOutput);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.kArmConnected = (arm.getFirmwareVersion() != 0);
    inputs.armAppliedVoltage = arm.getBusVoltage();
    inputs.pos = arm.getEncoder().getPosition();
    inputs.supplyArmCurrentAmps = arm.getOutputCurrent();
    inputs.velMetersPerSecond = arm.getEncoder().getVelocity();
  }

  @Override
  public void setArmPosition(double targetPosition) {
    arm.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);

    Logger.recordOutput("Manipulator/Arm/TargetPosition", targetPosition);
  }

  // @Override
  // public void seedPivotPosition(double newPositionMechanismRot) {
  //   armEncoder.setPosition(newPositionMechanismRot * Constants.Manipulator.Arm.motorGearRatio);
  // }

  @Override
  public void stop() {
    arm.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    armConfig.idleMode(IdleMode.kBrake);
    arm.configure(armConfig, null, null);
  }

  @Override
  public void enableCoastMode(boolean enable) {
    armConfig.idleMode(IdleMode.kCoast);
    arm.configure(armConfig, null, PersistMode.kPersistParameters);

    arm.clearFaults();
    armConfig.disableFollowerMode();
    armConfig.inverted(false);
    armConfig.smartCurrentLimit(Constants.Manipulator.Arm.currentLimit);
    armConfig.closedLoop.pid(
        Constants.Manipulator.Arm.kP, Constants.Manipulator.Arm.kI, Constants.Manipulator.Arm.kD);

    armConfig.closedLoop.outputRange(
        Constants.Manipulator.Arm.minOutput, Constants.Manipulator.Arm.maxOutput);

    arm.configure(armConfig, null, null);
  }
}
