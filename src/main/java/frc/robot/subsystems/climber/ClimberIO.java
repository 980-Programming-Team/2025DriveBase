package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {

    public double pos = 0.0;

    public boolean kClimber = false;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void stop() {}

  public default void setPosition(double position) {}

  public default void enableBrakeMode(boolean enable) {}
}
