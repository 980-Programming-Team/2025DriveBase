package frc.robot.subsystems.funnel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj.Alert;
import frc.robot.constants.Constants;

public class FunnelIOSpark implements FunnelIO {
  private SparkBase pivot;
  private SparkBase intake;
  private RelativeEncoder encoder;

  private SparkMaxConfig pivotConfig;
  private SparkMaxConfig intakeConfig;

  public FunnelIOSpark() {
    pivot = new SparkMax(Constants.Funnel.kFunnelPivot, MotorType.kBrushless);
    intake = new SparkMax(Constants.Funnel.kFunnelIntake, MotorType.kBrushless);

    pivotConfig = new SparkMaxConfig();
    intakeConfig = new SparkMaxConfig();

    configurePivot(pivot, pivotConfig);
    configureIntake(intake, intakeConfig);
  }

  private void configurePivot(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    motor.clearFaults();
    config.smartCurrentLimit(Constants.Funnel.Pivot.supplyCurrentLimit);
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.closedLoop.pid(
        Constants.Funnel.Pivot.kP, Constants.Funnel.Pivot.kI, Constants.Funnel.Pivot.kD);

    config.closedLoop.outputRange(
        Constants.Funnel.Pivot.minOutput, Constants.Funnel.Pivot.maxOutput);

    motor.configure(config, null, null);
  }

  private void configureIntake(SparkBase motor, SparkBaseConfig config) {

    motor.clearFaults();
    config.smartCurrentLimit(Constants.Funnel.Intake.supplyCurrentLimit);
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.pivotAppliedVoltage = pivot.getBusVoltage();
    inputs.pivotSupplyCurrentAmps = pivot.getOutputCurrent();
    inputs.pos = pivot.getEncoder().getPosition();
    inputs.intakeAppliedVoltage = intake.getBusVoltage();
    inputs.intakeSupplyCurrentAmps = intake.getOutputCurrent();
    inputs.intakeSpeedRotationsPerSec = intake.getEncoder().getVelocity();
  }

  @Override
  public void setPosition(double targetPosition) {
    pivot.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  // @Override
  // public void seedPivotPosition(double newPositionMechanismRot) {
  //   encoder.setPosition(newPositionMechanismRot * Constants.Funnel.Pivot.motorGearRatio);
  // }

  @Override
  public void enableBrakeMode(boolean enable) {
    pivotConfig.idleMode(IdleMode.kBrake);
    intakeConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void enableCoastMode(boolean enable) {
    pivotConfig.idleMode(IdleMode.kCoast);
    intakeConfig.idleMode(IdleMode.kCoast);
  }
}
