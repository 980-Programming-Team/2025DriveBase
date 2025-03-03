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
import org.littletonrobotics.junction.Logger;

public class ArmIOSpark implements ArmIO {
  private SparkBase arm;
  private SparkClosedLoopController armPIDController;
  private RelativeEncoder armEncoder;

  private SparkMaxConfig armConfig;

  private double targetPosition;

  public ArmIOSpark() {
    armConfig = new SparkMaxConfig();
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
    inputs.supplyArmCurrentAmps = arm.getOutputCurrent();
    inputs.armTempCelsius = arm.getMotorTemperature();
    inputs.armVelocity = arm.getEncoder().getVelocity();
  }

  @Override
  public void setArmPosition(double mechanismRotations) {
    targetPosition = mechanismRotations; // Constants.Elevator.gearRatio;
    armPIDController.setReference(targetPosition, ControlType.kPosition);

    Logger.recordOutput("Manipulator/Arm/TargetPosition", targetPosition);
    Logger.recordOutput("Manipulator/Arm/MechanismRotations", mechanismRotations);

    // Stop the motor when the target position is reached
    if ((arm.getEncoder().getPosition() - targetPosition)
        > Constants.Elevator.setpointToleranceMeters) {
      arm.stopMotor();
    }
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
