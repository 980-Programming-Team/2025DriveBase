package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.constants.Constants;

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
    config.smartCurrentLimit(Constants.Climber.supplyCurrentLimit);

    motor.configure(config, null, null);
  }

  private void configureFollower(SparkBase motor, SparkBaseConfig config) {
    
    config.follow(leader, false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.Climber.supplyCurrentLimit);

    motor.configure(config, null, null);
  }

  public double getPosition(){
    return encoder.getPosition();
  }  

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.posMeters = rotationsToMeters(leader.getEncoder().getPosition());
    inputs.velMetersPerSecond = rotationsToMeters(leader.getEncoder().getVelocity());

    inputs.kNearL1Connected = (leader.getFirmwareVersion() != 0);
    inputs.leaderAppliedVoltage = leader.getBusVoltage();
    inputs.supplyLeaderCurrentAmps = leader.getOutputCurrent();
    inputs.leaderTempCelcius = leader.getMotorTemperature();
    inputs.leaderPosMotorRotations = leader.getEncoder().getPosition();

    inputs.kNearFunnelConnected = (follower.getFirmwareVersion() != 0);
    inputs.followerTempCelcius = follower.getMotorTemperature();
  }

  private double rotationsToMeters(double rotations) {
    return rotations
        / Constants.Climber.gearRatio
        * (Math.PI * Constants.Climber.splineXLDiameter);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void setPosition(double newPositionMechanismRot) {
    encoder.setPosition(newPositionMechanismRot * Constants.Climber.gearRatio);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
