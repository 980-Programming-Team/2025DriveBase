package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSpark implements ArmIO {
  private SparkBase arm;
  private RelativeEncoder encoder;
  private DutyCycleEncoder throughBoreEncoder;

  private PIDController pidController;
  private SparkMaxConfig armConfig;

  public ArmIOSpark() {
    arm = new SparkMax(Constants.Manipulator.kArm, MotorType.kBrushless);

    armConfig = new SparkMaxConfig();

    configureArm(arm, armConfig);

    throughBoreEncoder = new DutyCycleEncoder(6, 5.0, 0.0); // 2.70 4.74
    throughBoreEncoder.setInverted(true);
    pidController =
        new PIDController(
            Constants.Manipulator.Arm.kP1.get(),
            Constants.Manipulator.Arm.kI1.get(),
            Constants.Manipulator.Arm.kD1.get());

    // pidController.setD(-Constants.Manipulator.Arm.kD1.get());
    // pidController.setTolerance(0);
  }

  // intake - 2.04
  // 2.20 l1 l2 2.36 l3 l4 4.03

  private void configureArm(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    motor.clearFaults();
    config.disableFollowerMode();
    config.inverted(true);
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
    inputs.absPos = throughBoreEncoder.get();
    inputs.supplyArmCurrentAmps = arm.getOutputCurrent();
    inputs.velMetersPerSecond = arm.getEncoder().getVelocity();
  }

  @Override
  public void setArmPosition(double targetPosition) {

    arm.set(pidController.calculate(throughBoreEncoder.get(), targetPosition));
    // arm.getClosedLoopController().setReference(targetPosition, ControlType.kDutyCycle);

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
