package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {

    public boolean kPivotConnected = false;
    public boolean kIntakeConnected = false;

    public double pivotAppliedVoltage = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pos = 0.0;
    public double velMetersPerSecond = 0.0;

    public double intakeAppliedVoltage = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeSpeedRotationsPerSec = 0.0;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setPosition(double targetPosition) {}

  public default void setIntakeVoltage(double voltage) {}

  public default void set(double speed) {}

  public default void setPivot(double speed) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}

  public default void enableCoastMode(boolean enable) {}
}
