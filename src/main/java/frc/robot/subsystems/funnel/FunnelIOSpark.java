package frc.robot.subsystems.funnel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.constants.Constants;

public class FunnelIOSpark implements FunnelIO {
  private SparkBase pivot;
  private SparkClosedLoopController pivotPIDController;
  private SparkBase intake;
  private RelativeEncoder pivotEncoder;
  // private Canandmag pivotEncoder;

  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();
  private Alert motorMissingAlert;

  public FunnelIOSpark() {
    pivot = new SparkMax(Constants.Funnel.kFunnelPivot, MotorType.kBrushless);
    intake = new SparkMax(Constants.Funnel.kFunnelIntake, MotorType.kBrushless);

    configurePivot(pivot, pivotConfig);
    configureIntake(intake, intakeConfig);

    pivotPIDController = pivot.getClosedLoopController();
    pivotEncoder = pivot.getEncoder();

    // pivotEncoder = new Encoder(Constants.Funnel.Pivot.EncoderDIO0,
    // Constants.Funnel.Pivot.EncoderDIO1);
  }

  private void configurePivot(SparkBase motor, SparkBaseConfig config) {

    config.smartCurrentLimit(Constants.Funnel.Pivot.supplyCurrentLimit);
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pidf(
        Constants.Funnel.Pivot.kP,
        Constants.Funnel.Pivot.kI,
        Constants.Funnel.Pivot.kD,
        Constants.Funnel.Pivot.kFF);

    config.closedLoop.outputRange(
        Constants.Funnel.Pivot.minOutput, Constants.Funnel.Pivot.maxOutput);

    motor.configure(config, null, null);
  }

  private void configureIntake(SparkBase motor, SparkBaseConfig config) {

    config.smartCurrentLimit(Constants.Funnel.Intake.supplyCurrentLimit);
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.pivotAppliedVoltage = pivot.getBusVoltage();
    inputs.pivotSupplyCurrentAmps = pivot.getOutputCurrent();
    inputs.pivotTempCelcius = pivot.getMotorTemperature();
    inputs.pivotPosMotorRotations = pivot.getEncoder().getPosition();
    inputs.pivotPosAbsMechanismRotations =
        (pivotEncoder.getPosition() > Constants.Funnel.Pivot.absZeroWrapThreshold)
            ? 0.0
            : (pivotEncoder.getPosition() / Constants.Funnel.Pivot.motorGearRatio);
    inputs.intakeAppliedVoltage = intake.getBusVoltage();
    inputs.intakeSupplyCurrentAmps = intake.getOutputCurrent();
    inputs.intakeTempCelcius = intake.getMotorTemperature();
    inputs.intakeSpeedRotationsPerSec = intake.getEncoder().getVelocity();
  }

  @Override
  public void setPivotPosition(double mechanismRotations) {
    double targetPosition = mechanismRotations * Constants.Funnel.Pivot.motorGearRatio;
    pivotPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  @Override
  public void seedPivotPosition(double newPositionMechanismRot) {
    pivotEncoder.setPosition(newPositionMechanismRot * Constants.Funnel.Pivot.motorGearRatio);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    pivotConfig.idleMode(IdleMode.kBrake);
    intakeConfig.idleMode(IdleMode.kBrake);
  }
}
