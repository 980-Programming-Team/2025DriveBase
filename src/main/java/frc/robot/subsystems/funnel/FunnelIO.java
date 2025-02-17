package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public double pivotAppliedVoltage = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotTempCelcius = 0.0;
    public double pivotPosMotorRotations = 0.0;
    public double pivotPosAbsMechanismRotations = 0.0;

    public double intakeAppliedVoltage = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeStatorCurrentAmps = 0.0;
    public double intakeTempCelcius = 0.0;
    public double intakeSpeedRotationsPerSec = 0.0;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setPivotPosition(double mechanismRotations) {}

  public default void setIntakeVoltage(double voltage) {}

  public default void seedPivotPosition(double newPositionMechanismRot) {}

  public default void enableBrakeMode(boolean enable) {}
}
