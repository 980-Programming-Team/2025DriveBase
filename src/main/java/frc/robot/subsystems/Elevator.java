package frc.robot.subsystems;

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
  private Drive Drivebase;
  private double Position;

  public Elevator(Drive drive) {
    ElevatorSpark = new SparkMax(12, MotorType.kBrushless);
    SparkBaseConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);

    Drivebase = drive;
  }

  @Override
  public void periodic() {
    Position = ElevatorSpark.getEncoder().getPosition();

    SmartDashboard.putNumber("elevatorPos", Position);
  }

  public void ManualUntilPos(double position) {
    if (Position <= position) ElevatorSpark.set(.3);
    else ElevatorSpark.set(0);
  }

  public void Reset() {
    if (Position >= 5) ElevatorSpark.set(-.3);
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
}
