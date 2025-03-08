package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  public FunnelIO io;
  public FunnelIOInputsAutoLogged inputs;

  // instead of has coeral check check if were in the range of a pose, robot

  private final Alert pivotMissingAlert;
  private final Alert intakeMissingAlert;

  private double setpoint;
  private FunnelStates state;

  public enum FunnelStates {
    STARTING_CONFIG,
    FEED,
    REQUEST_SETPOINT
  }

  public Funnel(FunnelIO funnelIO) {
    this.io = funnelIO;

    inputs = new FunnelIOInputsAutoLogged();

    pivotMissingAlert = new Alert("Disconnected Pivot Motor", AlertType.kError);
    intakeMissingAlert = new Alert("Disconnected Intake Motor", AlertType.kError);

    setpoint = 0;
    state = FunnelStates.STARTING_CONFIG;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);
    Logger.recordOutput("Funnel/State", state.toString());
    Logger.recordOutput("Funnel/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = FunnelStates.REQUEST_SETPOINT;
        }
        break;
      case FEED:
        break;
      case REQUEST_SETPOINT:
        if (setpoint != 0.0) {
          io.setPosition(setpoint);
        }
        break;
    }

    pivotMissingAlert.set(!inputs.kPivotConnected && Constants.currentMode != Mode.SIM);
    intakeMissingAlert.set(!inputs.kIntakeConnected && Constants.currentMode != Mode.SIM);
  }

  public void requestPosition(double position) {
    setpoint = position;
  }

  public double getPosition() {
    return inputs.pos;
  }

  public double getVelocity() {
    return inputs.velMetersPerSecond;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void enableCoastMode(boolean enable) {
    io.enableCoastMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
