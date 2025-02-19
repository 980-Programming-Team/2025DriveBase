package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;

public class ElevatorIOSpark implements ElevatorIO {
  private SparkBase leader;
  private SparkBase follower;
  private RelativeEncoder encoder;

  private SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private SparkMaxConfig followerConfig = new SparkMaxConfig();

  private ClosedLoopSlot L2Slot;
  private ClosedLoopSlot L4Slot;  

  //private ClosedLoopSlot slot2;


  public ElevatorIOSpark() {
    leader = new SparkMax(Constants.Elevator.kElevatorRoboRio, MotorType.kBrushless); // The leader is on the side of the robo rio
    follower = new SparkMax(Constants.Elevator.kElevatorPDH, MotorType.kBrushless); // The follower is on the side of the PDH

    configureLeader(leader, leaderConfig);
    configureFollower(follower, followerConfig);
  
  }

    motorConfigs.MotionMagic.MotionMagicAcceleration =
        (Constants.Elevator.mechanismMaxAccel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        (Constants.Elevator.mechanismMaxCruiseVel / (Math.PI * Constants.Elevator.sprocketDiameter))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicJerk = Constants.Elevator.motionMagicJerk;


    private void configureLeader(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();

    config.disableFollowerMode();
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Elevator.supplyCurrentLimit);
    config.closedLoop.pid(0.2, 0, 0.025, L2Slot);
    config.closedLoop.pid(0.4, 0, 0.05, L4Slot);
    config.closedLoop.outputRange(Constants.Elevator.peakReverse, Constants.Elevator.peakReverse);

    config.closedLoop.maxMotion.maxAcceleration(0, L2Slot);

    motor.configure(config, null, null);
  }

  private void configureFollower(SparkBase motor, SparkBaseConfig config) {
    
    config.follow(leader, true);
    config.idleMode(IdleMode.kBrake);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Elevator.supplyCurrentLimit);
    config.closedLoop.pid(0.2, 0, 0.025, L2Slot);
    config.closedLoop.pid(0.4, 0, 0.05, L4Slot);
    config.closedLoop.outputRange(Constants.Elevator.peakReverse, Constants.Elevator.peakReverse);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.posMeters = rotationsToMeters(leader.getEncoder().getPosition());
    inputs.velMetersPerSecond = rotationsToMeters(leader.getEncoder().getVelocity());
    inputs.appliedVoltage = leader.getBusVoltage();
    inputs.supplyCurrentAmps =
        new double[] {
          leader.getOutputCurrent(),
          follower.getOutputCurrent()
        };
    inputs.tempCelcius =
        new double[] {
          leader.getMotorTemperature(), follower.getMotorTemperature()
        };
  }

  @Override
  public void setHeight(double heightMeters) {
    if (heightMeters == Constants.Scoring.L2ScoringHeight) {
      leader.setControl(
          new MotionMagicVoltage(metersToRotations(heightMeters)).withSlot(1).withEnableFOC(true));
    } else if (heightMeters == Constants.Scoring.L3ScoringHeight) {
      leader.setControl(
          new MotionMagicVoltage(metersToRotations(heightMeters)).withSlot(2).withEnableFOC(true));
    } else {
      leader.setControl(
          new MotionMagicVoltage(metersToRotations(heightMeters)).withSlot(0).withEnableFOC(true));
    }
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
