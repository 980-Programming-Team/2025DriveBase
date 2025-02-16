package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmClawIO {
  @AutoLog
  public static class ArmClawIOInputs {
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double clawStatorCurrentAmps = 0.0;
    public double tempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;

    public boolean beamBreakTriggered = false;
  }

  public default void updateInputs(ArmClawIOInputs inputs) {}

  public default void setArmVoltage(double voltage) {}

  public default void setClawVoltage(double voltage) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
