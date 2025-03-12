package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSpark implements ClimberIO {
  private SparkBase leader;
  private SparkBase follower;

  // private Encoder throughBoreEncoder;
  private RelativeEncoder encoder;

  private SparkMaxConfig leaderConfig;
  private SparkMaxConfig followerConfig;

  public ClimberIOSpark() {
    leader =
        new SparkMax(
            Constants.Climber.kNearFunnel,
            MotorType.kBrushless); // The leader is on the side of the robo rio
    follower =
        new SparkMax(
            Constants.Climber.kNearL1,
            MotorType.kBrushless); // The follower is on the side of the PDH

    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();

    // throughBoreEncoder = new Encoder(Constants.Elevator.EncoderDIO2,
    // Constants.Elevator.EncoderDIO3);
    // throughBoreEncoder.reset();

    configureLeader(leader, leaderConfig);
    configureFollower(follower, followerConfig);
  }

  private void configureLeader(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    motor.clearFaults();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Climber.supplyCurrentLimit);
    config.closedLoop.pid(3, 0, 0.0);
    config.closedLoop.outputRange(Constants.Climber.minOutput, Constants.Climber.maxOutput);

    motor.configure(config, null, null);
  }

  private void configureFollower(SparkBase motor, SparkBaseConfig config) {

    motor.clearFaults();
    config.follow(leader, true);
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Climber.supplyCurrentLimit);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.pos = leader.getEncoder().getPosition();
    inputs.leaderAppliedVoltage = leader.getBusVoltage();
    inputs.supplyLeaderCurrentAmps = follower.getOutputCurrent();
  }

  @Override
  public void setPosition(double targetPosition) {
    leader.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
    // leader.setVoltage(heightMeters);
    Logger.recordOutput("Climber/TargetPosition", targetPosition);
  }

  // @Override
  // public void seedPosition(double motorPositionRot) {
  //   encoder.setPosition(motorPositionRot);
  // }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    leaderConfig.idleMode(IdleMode.kBrake);
    followerConfig.idleMode(IdleMode.kBrake);
  }
}
