package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;

public class FunnelIOSpark implements FunnelIO {
  private SparkBase pivot;
  private SparkBase intake;
  private Encoder pivotEncoder;
  // private Canandmag pivotEncoder;

  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();
  private Alert motorMissingAlert;

  public FunnelIOSpark() {
    pivot = new SparkMax(Constants.Funnel.kFunnelPivot, MotorType.kBrushless);
    intake = new SparkMax(Constants.Funnel.kFunnelIntake, MotorType.kBrushless);

    configurePivot(pivot);
    configureIntake(intake);
    StatusCode rollerConfigStatus = configRoller();

    motorMissingAlert = new Alert("NEO 550 " + getIntakeID() + " missing (Algae Roller)", AlertType.ERROR);

    if (rollerConfigStatus != StatusCode.OK) {
      motorMissingAlert.set(true);
    } else {
      motorMissingAlert.set(false);
    }

    pivotEncoder = new Encoder(Constants.Funnel.Pivot.pivotEncoderDIO1, Constants.Funnel.Pivot.pivotEncoderDIO2);
  }

  private void configurePivot(SparkBase motor) {
    motor.setSmartCurrentLimit(Constants.Funnel.Pivot.statorCurrentLimit);
    motor.setSecondaryCurrentLimit(Constants.Funnel.Pivot.supplyCurrentLimit);
  }

  private void configureIntake(SparkBase motor) {
    motor.setSmartCurrentLimit(Constants.Funnel.Pivot.statorCurrentLimit);
    motor.setSecondaryCurrentLimit(Constants.Funnel.Pivot.supplyCurrentLimit);
  }

  private int getPivotID(){
    return Constants.Funnel.kFunnelPivot;
  }

  private int getIntakeID(){
    return Constants.Funnel.kFunnelIntake;
  }

  private StatusCode configPivot() {
    pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Funnel.Pivot.statorCurrentLimit;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Funnel.Pivot.supplyCurrentLimit;

    pivotConfig.MotorOutput.Inverted = Constants.Flipper.Pivot.motorInversion;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.Slot0.kP = Constants.Flipper.Pivot.kP;
    pivotConfig.Slot0.kD = Constants.Flipper.Pivot.kD;

    pivotConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    pivotConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return pivotConfig.apply(pivotConfig);
  }

  private StatusCode configRoller() {
    rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Flipper.Roller.statorCurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Flipper.Roller.supplyCurrentLimit;

    rollerConfig.MotorOutput.Inverted = Constants.Flipper.Roller.motorInversion;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    rollerConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(FlipperIOInputs inputs) {
    inputs.pivotAppliedVoltage = pivot.getMotorVoltage().getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivot.getStatorCurrent().getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivot.getStatorCurrent().getValueAsDouble();
    inputs.pivotTempCelcius = pivot.getStatorCurrent().getValueAsDouble();
    inputs.pivotPosMotorRotations = pivot.getPosition().getValueAsDouble();
    inputs.pivotPosAbsMechanismRotations =
        (pivotEncoder.getAbsPosition() > Constants.Flipper.Pivot.absZeroWrapThreshold)
            ? 0.0
            : (pivotEncoder.getAbsPosition() / Constants.Flipper.Pivot.absEncoderGearRatio);
    inputs.rollerAppliedVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerStatorCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerTempCelcius = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerSpeedRotationsPerSec = rollerMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double mechanismRotations) {
    pivot.setControl(
        new PositionVoltage(mechanismRotations * Constants.Flipper.Pivot.motorGearRatio)
            .withEnableFOC(true));
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intake.setControl(new VoltageOut(voltage));
  }

  @Override
  public void seedPivotPosition(double newPositionMechanismRot) {
    pivot.setPosition(newPositionMechanismRot * Constants.Flipper.Pivot.motorGearRatio);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    pivot.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    rollerMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
