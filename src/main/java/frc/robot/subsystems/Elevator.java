package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Elevator extends SubsystemBase {
  private SparkMax ElevatorSpark;
  private RelativeEncoder ElevatorEncoder;
  private SparkBaseConfig Config;

  private Drive Drivebase;

  public Elevator(Drive drive) {
    ElevatorSpark = new SparkMax(12, MotorType.kBrushless);
    ElevatorEncoder = ElevatorSpark.getEncoder();
    Config = new SparkFlexConfig();
    Config.idleMode(IdleMode.kBrake);
    ElevatorEncoder.setPosition(0);

    Drivebase = drive;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pos", ElevatorEncoder.getPosition());
  }

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
