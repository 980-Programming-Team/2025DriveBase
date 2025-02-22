package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;

public class ElevatorIOSpark implements ElevatorIO {
  private SparkBase leader;
  private SparkClosedLoopController leaderPIDController;
  private SparkBase follower;
  private Encoder throughBoreEncoder;
  private RelativeEncoder encoder;

  private SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private SparkMaxConfig followerConfig = new SparkMaxConfig();

  // private ClosedLoopSlot slot2;

  public ElevatorIOSpark() {
    leader =
        new SparkMax(
            Constants.Elevator.kElevatorRoboRio,
            MotorType.kBrushless); // The leader is on the side of the robo rio
    follower =
        new SparkMax(
            Constants.Elevator.kElevatorPDH,
            MotorType.kBrushless); // The follower is on the side of the PDH

    // throughBoreEncoder = new Encoder(Constants.Elevator.EncoderDIO2,
    // Constants.Elevator.EncoderDIO3);
    // throughBoreEncoder.reset();

    leaderPIDController = leader.getClosedLoopController();

    configureLeader(leader, leaderConfig);
    configureFollower(follower, followerConfig);
  }

  private void configureLeader(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    config.disableFollowerMode();
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Elevator.supplyCurrentLimit);
    config.closedLoop.pid(0.2, 0, 0.025);
    config.closedLoop.outputRange(Constants.Elevator.peakReverse, Constants.Elevator.peakReverse);

    config.closedLoop.maxMotion.maxAcceleration(
        (Constants.Elevator.mechanismMaxAccel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio);
    config.closedLoop.maxMotion.maxVelocity(
        (Constants.Elevator.mechanismMaxCruiseVel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio);

    motor.configure(config, null, null);
  }

  private void configureFollower(SparkBase motor, SparkBaseConfig config) {

    config.follow(leader, true);
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Elevator.supplyCurrentLimit);
    config.closedLoop.pid(0.2, 0, 0.025);
    config.closedLoop.outputRange(Constants.Elevator.peakReverse, Constants.Elevator.peakReverse);

    config.closedLoop.maxMotion.maxAcceleration(
        (Constants.Elevator.mechanismMaxAccel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio);
    config.closedLoop.maxMotion.maxVelocity(
        (Constants.Elevator.mechanismMaxCruiseVel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.posMeters = rotationsToMeters(leader.getEncoder().getPosition());
    inputs.velMetersPerSecond =
        rotationsToMeters(leader.getEncoder().getVelocity()); // throughBoreEncoder.getRate()
    inputs.appliedVoltage = leader.getBusVoltage();
    inputs.supplyCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.tempCelcius =
        new double[] {leader.getMotorTemperature(), follower.getMotorTemperature()};
  }

  @Override
  public void setHeight(double heightMeters) {
    double targetPosition = heightMeters * Constants.Elevator.gearRatio;
    leaderPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void seedPosition(double motorPostionRot) {
    encoder.setPosition(motorPostionRot);
  }

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

  private double metersToRotations(double heightMeters) {
    return (heightMeters / (Math.PI * Constants.Elevator.sprocketDiameter))
        * Constants.Elevator.gearRatio;
  }

  private double rotationsToMeters(double rotations) {
    return rotations
        / Constants.Elevator.gearRatio
        * (Math.PI * Constants.Elevator.sprocketDiameter);
  }
}
