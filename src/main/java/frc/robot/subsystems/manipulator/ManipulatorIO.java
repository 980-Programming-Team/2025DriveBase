package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
  @AutoLog
  public static class ManipulatorIOInputs {
    public boolean armConnected = false;
    public double armAppliedVoltage = 0.0;
    public double supplyArmCurrentAmps = 0.0;
    public double armTempCelcius = 0.0;
    public double armSpeedRotationsPerSec = 0.0;

    public boolean clawConnected = false;
    public double clawAppliedVoltage = 0.0;
    public double supplyClawCurrentAmps = 0.0;
    public double clawTempCelcius = 0.0;
    public double clawSpeedRotationsPerSec = 0.0;

    public boolean beamBreakTriggered = false;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setArmVoltage(double voltage) {}

  public default void setClawVoltage(double voltage) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
