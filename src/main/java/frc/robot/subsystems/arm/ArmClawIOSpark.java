package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class ArmClawIOSpark implements ArmClawIO {
  private SparkBase arm;
  private SparkBase claw;
  //private Canandcolor beamBreak;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public ArmClawIOSpark() {
    arm = new SparkMax(Constants.Arm.kArm, MotorType.kBrushless);
    claw = new SparkMax(Constants.Arm.kClaw, MotorType.kBrushless);

    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Arm.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Arm.supplyCurrentLimit;

    motorConfigs.MotorOutput.Inverted = Constants.Arm.motorInversion;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode configStatus = motor.getConfigurator().apply(motorConfigs);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + motor.getDeviceID()
              + " error (End Effector): "
              + configStatus.getDescription(),
          false);
    }

    //beamBreak = new Canandcolor(Constants.EndEffector.frontBeamBreakID);

    // TODO: Set up config to use near-zero latency described in Redux docs
  }

  @Override
  public void updateInputs(ArmClawIOInputs inputs) {
    inputs.appliedVoltage = arm.getMotorVoltage().getValueAsDouble();
    inputs.speedRotationsPerSec = arm.getVelocity().getValueAsDouble();
    inputs.statorCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = arm.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelcius = arm.getDeviceTemp().getValueAsDouble();
    // inputs.frontBeamBreakTriggered =
    //     beamBreak.getProximity() < Constants.Arm.proximityDetectionThreshold;
  }

  @Override
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  @Override
  public void stop() {
    arm.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    arm.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
