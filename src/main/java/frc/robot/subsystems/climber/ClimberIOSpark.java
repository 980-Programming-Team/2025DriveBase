package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;
import frc.robot.subsystems.funnel.FunnelIO.FunnelIOInputs;

public class ClimberIOSpark implements ClimberIO {
  private SparkBase leader;
  private SparkBase follower;
  private RelativeEncoder encoder;

  private SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private SparkMaxConfig followerConfig = new SparkMaxConfig();


  public ClimberIOSpark() {
    leader = new SparkMax(Constants.Climber.kNearL1, MotorType.kBrushless); // The leader is NEAR L1 MECHANISM
    follower = new SparkMax(Constants.Climber.kNearFunnel, MotorType.kBrushless); // The follower is NEAR the CAGE FUNNEL

    configureLeader(leader, leaderConfig);
    configureFollower(follower, followerConfig);

  }

  private void configureLeader(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();

    config.disableFollowerMode();
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, null, null);
  }

  private void configureFollower(SparkBase motor, SparkBaseConfig config) {
    
    config.follow(leader, false);
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, null, null);
  }

  public double getPosition(){
    return encoder.getPosition();
  }  

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.kNearL1Connected = (leader.getFirmwareVersion() != 0);
    inputs.leaderAppliedVoltage = leader.getBusVoltage();
    inputs.supplyLeaderCurrentAmps = leader.getOutputCurrent();
    inputs.leaderTempCelcius = leader.getMotorTemperature();
    inputs.leaderPosMotorRotations = leader.getEncoder().getPosition();

    inputs.kNearFunnelConnected = (follower.getFirmwareVersion() != 0);
    inputs.followerAppliedVoltage = follower.getBusVoltage();
    inputs.supplyFollowerCurrentAmps = follower.getOutputCurrent();
    inputs.followerTempCelcius = follower.getMotorTemperature();
    inputs.followerPosMotorRotations = follower.getEncoder().getVelocity();
  }

  @Override
  public void setClimberPosition(double mechanismRotations) {
    leader.setControl(
        new PositionVoltage(mechanismRotations * Constants.Flipper.Pivot.motorGearRatio)
            .withEnableFOC(true));
  }

  @Override
  public void setClimberVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void seedPivotPosition(double newPositionMechanismRot) {
    pivot.setPosition(newPositionMechanismRot * Constants.Flipper.Pivot.motorGearRatio);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
