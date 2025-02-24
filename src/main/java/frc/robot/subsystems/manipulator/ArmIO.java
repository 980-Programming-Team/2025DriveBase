package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean kArmConnected = false;
    public double armAppliedVoltage = 0.0;
    public double supplyArmCurrentAmps = 0.0;
    public double armTempCelcius = 0.0;
    public double armPosMotorRotations = 0.0;
    public double armPosAbsMechanismRotations = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmPosition(double mechanismRotations) {}

  public default void seedPivotPosition(double newPositionMechanismRot) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
