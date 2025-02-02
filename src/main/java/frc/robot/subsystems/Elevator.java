package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Elevator extends SubsystemBase {
  private SparkMax ElevatorSpark;
  private RelativeEncoder ElevatorEncoder;
  private SparkBaseConfig Config;

  private SwerveSubsystem Drivebase;

  public Elevator(SwerveSubsystem drive) {
    ElevatorSpark = new SparkMax(Constants.elevator, MotorType.kBrushless);
    ElevatorEncoder = ElevatorSpark.getEncoder();
    Config = new SparkFlexConfig();
    Config.idleMode(IdleMode.kBrake);
    ElevatorEncoder.setPosition(0);

    Drivebase = drive;
  }

  @Override
  public void periodic() {}

  public void Manual(double speed) {
    if (speed > .1 && ElevatorEncoder.getPosition() <= 80) ElevatorSpark.set(speed);
    else ElevatorSpark.set(0);
  }

  public void ManualUntilPos(double position) {
    if (ElevatorEncoder.getPosition() <= position) ElevatorSpark.set(.3);
    else ElevatorSpark.set(0);
  }

  public void Reset() {
    if (ElevatorEncoder.getPosition() >= 5) ElevatorSpark.set(-.7);
    else ElevatorSpark.set(0);
  }

  // public void Stowed() {
  //   final double kTolerance = 2.0;
  //   double targetPosition = 40;

  //   // Set output range for closed-loop controller
  //   Config.closedLoop.outputRange(-0.3, 0.3);

  //   // Set target position for the elevator in the closed-loop controller
  //   ElevatorSpark.getClosedLoopController()
  //       .setReference(targetPosition, SparkMax.ControlType.kPosition);

  //   // Get current position of the elevator
  //   double currentPosition = ElevatorEncoder.getPosition();

  //   // Check if we are within tolerance
  //   if (Math.abs(currentPosition - targetPosition) <= kTolerance) {
  //     // Stop the elevator if within tolerance
  //     ElevatorSpark.set(0);
  //   }
  // }

  // public double getCurrentPosition() {
  //   return ElevatorEncoder.getPosition();
  // }

  public void Auto(int level) {
    switch (level) {
      case 1:
        ManualUntilPos(20);
        break;
      case 2:
        ManualUntilPos(40);
        break;
      case 3:
        ManualUntilPos(60);
        break;
      case 4:
        ManualUntilPos(80);
        break;
    }
  }

  public double GetEncPosition() {
    return ElevatorEncoder.getPosition();
  }
}
