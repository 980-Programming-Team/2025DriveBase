package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public ClimberIO io;
  public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final Alert leaderMissingAlert =
      new Alert("Disconnected Near L1 Mechanism Climber Motor", AlertType.kError);
  private final Alert followerMissingAlert =
      new Alert("Disconnected Near Funnel Climber Motor", AlertType.kError);

  private double setpoint = 0;
  private ClimberStates state = ClimberStates.STARTING_CONFIG;

  public enum ClimberStates {
    STARTING_CONFIG,
    STOWED,
    CLIMBING
  }

  public Climber(ClimberIO climberIO) {
    this.io = climberIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = ClimberStates.STOWED;
        }
        break;
      case STOWED:
        io.setPosition(setpoint);
        break;
      case CLIMBING:
        io.setPosition(setpoint);
        break;
    }

    leaderMissingAlert.set(!inputs.kNearL1Connected && Constants.currentMode != Mode.SIM);
    followerMissingAlert.set(!inputs.kNearFunnelConnected && Constants.currentMode != Mode.SIM);
  }

  public void requestIdle(double positionMeters) {
    state = ClimberStates.STOWED;
    setpoint = positionMeters;
  }

  public void requestClimbingPosition(double positionMeters) {
    state = ClimberStates.CLIMBING;
    setpoint = positionMeters;
  }

  public double getPosition() {
    return inputs.posMeters;
  }

  public double getVelocity() {
    return inputs.velMetersPerSecond;
  }

  public boolean atSetpoint() {
    return Util.atReference(
        inputs.posMeters, setpoint, Constants.Climber.setpointToleranceMeters, true);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
