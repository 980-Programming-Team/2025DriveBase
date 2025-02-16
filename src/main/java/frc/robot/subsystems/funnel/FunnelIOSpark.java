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

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class FunnelIOSpark implements FunnelIO {
  private SparkBase pivot;
  private SparkBase intake;
  // private Canandmag pivotEncoder;

  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();

  public FunnelIOSpark() {
    pivot = new SparkMax(Constants.Funnel.kFunnelPivot, MotorType.kBrushless);
    intake = new SparkMax(Constants.Funnel.kFunnelIntake, MotorType.kBrushless);

    StatusCode pivotConfigStatus = configPivot();
    StatusCode rollerConfigStatus = configRoller();

    if (pivotConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "NEO "
              + pivot.getDeviceID()
              + " error (Algae Pivot): "
              + pivotConfigStatus.getDescription(),
          false);
    }

    if (rollerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "NEO 550 "
              + intake.getDeviceID()
              + " error (Algae Roller): "
              + rollerConfigStatus.getDescription(),
          false);
    }

    pivotEncoder = new Canandmag(Constants.Flipper.pivotEncoderID);

    // TODO: Set configs for encoder referencing Redux docs
  }

  private StatusCode configPivot() {
    pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Flipper.Pivot.statorCurrentLimit;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Flipper.Pivot.supplyCurrentLimit;

    pivotConfig.MotorOutput.Inverted = Constants.Flipper.Pivot.motorInversion;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.Slot0.kP = Constants.Flipper.Pivot.kP;
    pivotConfig.Slot0.kD = Constants.Flipper.Pivot.kD;

    pivotConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    pivotConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return pivot.getConfigurator().apply(pivotConfig);
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
  public void setRollerVoltage(double voltage) {
    rollerMotor.setControl(new VoltageOut(voltage));
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
