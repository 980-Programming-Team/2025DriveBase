package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class Collector extends SubsystemBase {
  private SparkMax CollectorSpark;
  private RelativeEncoder CollectorEncoder;
  private SparkBaseConfig CollectorConfig;

  private SparkMax IndexSpark;
  private RelativeEncoder IndexEncoder;
  private SparkBaseConfig IndexConfig;

  public Collector(Drive drive) {
    CollectorSpark = new SparkMax(Constants.collect, MotorType.kBrushless);
    CollectorEncoder = CollectorSpark.getEncoder();
    CollectorConfig = new SparkFlexConfig();
    CollectorConfig.idleMode(IdleMode.kBrake);
    CollectorEncoder.setPosition(0);

    IndexSpark = new SparkMax(Constants.index, MotorType.kBrushless);
    IndexEncoder = IndexSpark.getEncoder();
    IndexConfig = new SparkFlexConfig();
    IndexConfig.idleMode(IdleMode.kBrake);
    IndexEncoder.setPosition(0);
  }

  @Override
  public void periodic() {}

  public void outtake() {
    CollectorSpark.set(-.5);
    IndexSpark.set(.3);
  }

  public void intake() {
    CollectorSpark.set(.5);
  }

  public void index() {
    IndexSpark.set(.5);
  }

  public void indexIntoShooter() {
    CollectorSpark.set(.5);
    IndexSpark.set(-.3);
  }

  public void indexIntoElevator() {
    CollectorSpark.set(.5);
    IndexSpark.set(.3);
  }

  public void off() {
    CollectorSpark.set(0);
    IndexSpark.set(0);
  }
}
