package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public boolean kClawConnected = false;
    public double clawAppliedVoltage = 0.0;
    public double supplyClawCurrentAmps = 0.0;
    public double clawTempCelcius = 0.0;
    public double clawSpeedRotationsPerSec = 0.0;
  }

  public default void updateInputs(ClawIOInputs inputs) {}

  public default void setClawVoltage(double voltage) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
