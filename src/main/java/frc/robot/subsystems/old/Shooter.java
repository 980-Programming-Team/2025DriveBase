// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.old;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends SubsystemBase {
  private final double SHOOTER_RPM_MAX = 5800; // /60 to make rpm to rps,  TODO: get actual max RPM
  private final double SHOOTER_FEEDFORWARD_KS = 0.05; // probably the same as before
  private final double SHOOTER_FEEDFORWARD_KV = 12.0 / SHOOTER_RPM_MAX;

  private SparkMax flywheelRightBot;
  private SparkMax flywheelLeftTop;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;
  private SparkBaseConfig RightBotConfig;
  private SparkBaseConfig LeftTopConfig;

  private PIDController topControl;
  private PIDController bottomControl;
  private SimpleMotorFeedforward ff;

  private double topShooterRowSpeed;
  private double bottomShooterRowSpeed;
  private boolean reachedMaxSpeed;

  /** Creates a new Shooter. */
  public Shooter(Drive drive) {
    flywheelRightBot = new SparkMax(Constants.flywheelRightBot, MotorType.kBrushless);
    RightBotConfig = new SparkFlexConfig();
    RightBotConfig.inverted(true);

    flywheelLeftTop = new SparkMax(Constants.flywheelLeftTop, MotorType.kBrushless);
    LeftTopConfig = new SparkFlexConfig();
    LeftTopConfig.inverted(true);

    topEncoder = flywheelRightBot.getEncoder();
    topControl = new PIDController(.001, 0, 0);

    bottomEncoder = flywheelLeftTop.getEncoder();
    bottomControl = new PIDController(.002, 0, 0);

    ff = new SimpleMotorFeedforward(SHOOTER_FEEDFORWARD_KS, SHOOTER_FEEDFORWARD_KV);

    topShooterRowSpeed = 0;
    bottomShooterRowSpeed = 0;
    reachedMaxSpeed = false;
  }

  @Override
  public void periodic() {
    topShooterRowSpeed = topEncoder.getVelocity();
    bottomShooterRowSpeed = bottomEncoder.getVelocity();

    // SmartDashboard.putNumber("leftAngle", angleLeft.getEncoder().getPosition());
    // SmartDashboard.putNumber("rightAngle", angleRight.getEncoder().getPosition());
    SmartDashboard.putNumber("speed", 0);
  }

  public void fireNote(double upper, double lower) {
    double setpoint = upper; // <30  2000 4000
    double setpoint2 = lower; // 30-36 2000 2000

    flywheelRightBot.setVoltage(
        topControl.calculate(topShooterRowSpeed, setpoint) + ff.calculate(setpoint));
    flywheelLeftTop.setVoltage(
        bottomControl.calculate(bottomShooterRowSpeed, setpoint2) + ff.calculate(setpoint2));
  }

  public void off() {
    flywheelLeftTop.setVoltage(0);
    flywheelRightBot.setVoltage(0);
  }
}
/*
  public void tiltShooter(double speed){
    // if
    // (
    //   (angleLeft.getEncoder().getPosition() > 0  && angleRight.getEncoder().getPosition() > 0  && speed > 0) ||
    //   (angleLeft.getEncoder().getPosition() < 39 && angleRight.getEncoder().getPosition() < 39 && speed < 0)
    // )
    // {
      if (Math.abs(speed) > .1)
      {
        angleLeft.set(speed * .2);
        angleRight.set(speed * .2);

        SmartDashboard.putNumber("speed", speed);
      }
      else
      {
        angleLeft.set(0);
        angleLeft.set(0);
      }
    }
  // }
}
*/
