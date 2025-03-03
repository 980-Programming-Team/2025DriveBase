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
// import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSpark implements ElevatorIO {
  private SparkBase leader;
  private SparkClosedLoopController leaderPIDController;
  private SparkBase follower;
  // private Encoder throughBoreEncoder;
  private RelativeEncoder encoder;

  private SparkMaxConfig leaderConfig;
  private SparkMaxConfig followerConfig;

  private double targetPosition;

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

    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();

    // throughBoreEncoder = new Encoder(Constants.Elevator.EncoderDIO2,
    // Constants.Elevator.EncoderDIO3);
    // throughBoreEncoder.reset();

    configureLeader(leader, leaderConfig);
    configureFollower(follower, followerConfig);

    leaderPIDController = leader.getClosedLoopController();
  }

  private void configureLeader(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    config.disableFollowerMode();
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Elevator.supplyCurrentLimit);
    config.closedLoop.pidf(0.8, 0, 0.15, 0.15);
    config.closedLoop.outputRange(Constants.Elevator.peakReverse, Constants.Elevator.peakReverse);

    // config.closedLoop.maxMotion.maxAcceleration(
    //     (Constants.Elevator.mechanismMaxAccel / (Math.PI * Constants.Elevator.sprocketDiameter))
    //         * Constants.Elevator.gearRatio);
    // config.closedLoop.maxMotion.maxVelocity(
    //     (Constants.Elevator.mechanismMaxCruiseVel / (Math.PI *
    // Constants.Elevator.sprocketDiameter))
    //         * Constants.Elevator.gearRatio);

    motor.configure(config, null, null);
  }

  private void configureFollower(SparkBase motor, SparkBaseConfig config) {

    config.follow(leader, true);
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Elevator.supplyCurrentLimit);
    config.closedLoop.pidf(0.8, 0, 0.15, 0.15);
    config.closedLoop.outputRange(Constants.Elevator.peakReverse, Constants.Elevator.peakReverse);

    // config.closedLoop.maxMotion.maxAcceleration(
    //     (Constants.Elevator.mechanismMaxAccel / (Math.PI * Constants.Elevator.sprocketDiameter))
    //         * Constants.Elevator.gearRatio);
    // config.closedLoop.maxMotion.maxVelocity(
    //     (Constants.Elevator.mechanismMaxCruiseVel / (Math.PI *
    // Constants.Elevator.sprocketDiameter))
    //         * Constants.Elevator.gearRatio);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.kRoborioMotorConnected = (leader.getFirmwareVersion() != 0);
    inputs.kPDHMotorConnected = (leader.getFirmwareVersion() != 0);
    inputs.posMeters = rotationsToMeters(leader.getEncoder().getPosition());
    inputs.pos = leader.getEncoder().getPosition();
    inputs.velMetersPerSecond =
        rotationsToMeters(leader.getEncoder().getVelocity()); // throughBoreEncoder.getRate()
    inputs.appliedVoltage = leader.getBusVoltage();
    inputs.supplyCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.tempCelsius =
        new double[] {leader.getMotorTemperature(), follower.getMotorTemperature()};
  }

  @Override
  public void setHeight(double heightMeters) {
    targetPosition = heightMeters * -1; // Constants.Elevator.gearRatio;
    leaderPIDController.setReference(targetPosition, ControlType.kPosition);

    Logger.recordOutput("Elevator/TargetPosition", targetPosition);
    Logger.recordOutput("Elevator/HeightMeters", heightMeters);

    // Stop the motor when the target position is reached
    if ((leader.getEncoder().getPosition() - targetPosition)
        < Constants.Elevator.setpointToleranceMeters) {
      leader.stopMotor();
      follower.stopMotor();
    }
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
    follower.setVoltage(voltage);
  }

  @Override
  public void seedPosition(double motorPositionRot) {
    encoder.setPosition(motorPositionRot);
  }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  // if (leader.getEncoder().getPosition() > targetPosition) {
  //   leader.set(-.5);
  //   // follower.set(5);
  // } else {
  //   leader.set(0);
  //   // follower.set(0);
  // }

  @Override
  public void enableBrakeMode(boolean enable) {
    leaderConfig.idleMode(IdleMode.kBrake);
    followerConfig.idleMode(IdleMode.kBrake);
  }

  // private double metersToRotations(double heightMeters) {
  //   return (heightMeters / (Math.PI * Constants.Elevator.sprocketDiameter))
  //       * Constants.Elevator.gearRatio;
  // }

  private double rotationsToMeters(double rotations) {
    return rotations
        / Constants.Elevator.gearRatio
        * (Math.PI * Constants.Elevator.sprocketDiameter);
  }
}
