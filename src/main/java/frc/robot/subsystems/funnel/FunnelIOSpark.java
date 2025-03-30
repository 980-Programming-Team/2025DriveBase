package frc.robot.subsystems.funnel;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj.Alert;
import frc.robot.constants.Constants;

public class FunnelIOSpark implements FunnelIO {
  private SparkBase intake;

  private SparkMaxConfig intakeConfig;

  public FunnelIOSpark() {
    intake = new SparkMax(Constants.Funnel.kFunnelIntake, MotorType.kBrushless);
    intakeConfig = new SparkMaxConfig();

    configureIntake(intake, intakeConfig);
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
    inputs.intakeAppliedVoltage = intake.getBusVoltage();
    inputs.intakeSupplyCurrentAmps = intake.getOutputCurrent();
    inputs.intakeSpeedRotationsPerSec = intake.getEncoder().getVelocity();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  @Override
  public void set(double speed) {
    intake.set(speed);
  }

  // @Override
  // public void seedPivotPosition(double newPositionMechanismRot) {
  //   encoder.setPosition(newPositionMechanismRot * Constants.Funnel.Pivot.motorGearRatio);
  // }

  @Override
  public void enableBrakeMode(boolean enable) {
    intakeConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void enableCoastMode(boolean enable) {
    intakeConfig.idleMode(IdleMode.kCoast);
  }
}
