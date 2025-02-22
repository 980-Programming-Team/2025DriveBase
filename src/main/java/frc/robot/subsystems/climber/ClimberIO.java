package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {

    public double posMeters = 0.0;
    public double velMetersPerSecond = 0.0;

    public boolean kNearL1Connected = false;
    public double leaderAppliedVoltage = 0.0;
    public double supplyLeaderCurrentAmps = 0.0;
    public double leaderTempCelcius = 0.0;
    public double leaderPosMotorRotations = 0.0;

    public boolean kNearFunnelConnected = false;
    public double followerTempCelcius = 0.0;
    // public double followerAppliedVoltage = 0.0;
    // public double supplyFollowerCurrentAmps = 0.0;
    // public double followerPosMotorRotations = 0.0;

  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stop() {}

  public default void setPosition(double motorPositionRot) {}

  public default void enableBrakeMode(boolean enable) {}
}
