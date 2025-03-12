package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public ClimberIO io;
  public ClimberIOInputsAutoLogged inputs;

  private final Alert leaderMissingAlert;
  private final Alert followerMissingAlert;

  private double setpoint;
  private ClimberStates state;

  public enum ClimberStates {
    STARTING_CONFIG,
    REQUEST_SETPOINT
  }

  public Climber(ClimberIO climberIO) {
    this.io = climberIO;

    inputs = new ClimberIOInputsAutoLogged();

    leaderMissingAlert = new Alert("Disconnected Climber Funnel Motor", AlertType.kError);
    followerMissingAlert = new Alert("Disconnected Climber L1 Motor", AlertType.kError);

    setpoint = 0;
    state = ClimberStates.STARTING_CONFIG;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/State", state.toString());
    Logger.recordOutput("Climber/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = ClimberStates.REQUEST_SETPOINT;
        }
        break;
      case REQUEST_SETPOINT:
        if (setpoint != 0.0) {
          io.setPosition(setpoint);
        }
        break;
    }

    leaderMissingAlert.set(!inputs.kNearFunnelConnected && Constants.currentMode != Mode.SIM);
    followerMissingAlert.set(!inputs.kNearL1Connected && Constants.currentMode != Mode.SIM);
  }

  public void requestPosition(double position) {
    setpoint = position;
  }

  public double getPosition() {
    return inputs.pos;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
