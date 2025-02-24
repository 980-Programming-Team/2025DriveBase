package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  private ClawIO io;
  private ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  private final Alert clawMissingAlert = new Alert("Disconnected Claw Motor", AlertType.kError);

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestShoot;
  private boolean requestShootL2;

  private boolean coralSecured;
  private ClawStates state = ClawStates.IDLE;

  private Timer shootTimer = new Timer();
  private Timer homingTimer = new Timer();

  public enum ClawStates {
    IDLE,
    FEED,
    SHOOTL2,
    SHOOT
  }

  public Claw(ClawIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput("Manipulator/Coral Detection", hasCoral());

    switch (state) {
      case IDLE:
        io.stop();

        // reset coral secured in cases where coral is removed manually from robot
        if (!hasCoral()) {
          coralSecured = false;
        }

        if (requestFeed && !coralSecured()) {
          state = ClawStates.FEED;
        } else if (requestShoot && coralSecured) {
          state = ClawStates.SHOOT;
        } else if (requestShootL2 && coralSecured) {
          state = ClawStates.SHOOTL2;
        }
        break;
      case FEED:
        io.setClawVoltage(Constants.Manipulator.Claw.feedVoltage);

        if (coralSecured()) {
          state = ClawStates.IDLE;
        } else if (requestIdle) {
          state = ClawStates.IDLE;
        }
        break;
      case SHOOTL2:
        io.setClawVoltage(Constants.Manipulator.Claw.scoreL2Voltage);

        if (!coralSecured()) {
          state = ClawStates.IDLE;
        }
        break;
      case SHOOT:
        io.setClawVoltage(Constants.Manipulator.Claw.scoreVoltage);

        if (!coralSecured()) {
          state = ClawStates.IDLE;
        }
        break;
    }

    clawMissingAlert.set(!inputs.kClawConnected && Constants.currentMode != Mode.SIM);
  }

  public boolean hasCoral() {
    // Assuming resistance can be inferred from the current draw
    double currentDraw = inputs.supplyClawCurrentAmps;
    return currentDraw > Constants.Manipulator.Claw.coralDetectionCurrentThreshold;

    //    return inputs.frontBeamBreakTriggered || inputs.backBeamBreakTriggered;
  }

  public boolean coralSecured() {
    coralSecured = hasCoral();
    return coralSecured;
  }

  // Use method only to reset state when robot is disabled
  public void forceIdle() {
    unsetAllRequests();
    state = ClawStates.IDLE;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void requestShoot() {
    unsetAllRequests();
    requestShoot = true;
  }

  public void requestShootL2() {
    unsetAllRequests();
    requestShootL2 = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestFeed = false;
    requestShootL2 = false;
    requestShoot = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
