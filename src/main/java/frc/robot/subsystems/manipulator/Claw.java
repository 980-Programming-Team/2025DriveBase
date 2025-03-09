package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  public ClawIO io;
  private ClawIOInputsAutoLogged inputs;

  private final Alert clawMissingAlert;

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestShoot;
  private boolean requestShootL2;

  private boolean coralSecured;
  private ClawStates state;

  private Timer shootTimer;
  // private Timer homingTimer;

  public enum ClawStates {
    IDLE,
    FEED,
    SHOOTL2,
    SHOOT
  }

  public Claw(ClawIO io) {
    this.io = io;

    inputs = new ClawIOInputsAutoLogged();
    clawMissingAlert = new Alert("Disconnected Claw Motor", AlertType.kError);
    state = ClawStates.IDLE;
    shootTimer = new Timer();
    shootTimer.stop();
    shootTimer.reset();
    // homingTimer = new Timer();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput("Manipulator/Coral Detection", hasCoral());
    Logger.recordOutput("Manipulator/Timer", shootTimer.get());
    Logger.recordOutput("Manipulator/ClawState", state.toString());

    switch (state) {
      case IDLE:
        io.stop();

        // reset coral secured in cases where coral is removed manually from robot
        if (!hasCoral()) {
          coralSecured = false;
        }

        if (requestFeed && shootTimer.get() <= 0 /*&& !coralSecured()*/) {
          state = ClawStates.FEED;
        } else if (requestShoot && shootTimer.get() <= 0 /*&& coralSecured*/) {
          state = ClawStates.SHOOT;
        } else if (requestShootL2 && shootTimer.get() <= 0 /*&& coralSecured/* */) {
          state = ClawStates.SHOOTL2;
        }
        break;
      case FEED:
        io.setClawSpeed(Constants.Manipulator.Claw.feedSpeed);

        if (shootTimer.get() <= 0) shootTimer.start();

        if (shootTimer.get() >= 3 || requestIdle) {
          state = ClawStates.IDLE;
          shootTimer.stop();
          shootTimer.reset();
          requestIdle();
        }
        break;
      case SHOOTL2:
        io.setClawSpeed(Constants.Manipulator.Claw.scoreL2Speed);

        if (shootTimer.get() <= 0) shootTimer.start();

        if (shootTimer.get() >= 2 || requestIdle) {
          state = ClawStates.IDLE;
          shootTimer.stop();
          shootTimer.reset();
          requestIdle();
        }
        break;
      case SHOOT:
        io.setClawSpeed(Constants.Manipulator.Claw.scoreSpeed);

        if (shootTimer.get() <= 0) shootTimer.start();

        if (shootTimer.get() >= 2 || requestIdle) {
          state = ClawStates.IDLE;
          shootTimer.stop();
          shootTimer.reset();
          requestIdle();
        }
        break;
    }

    clawMissingAlert.set(!inputs.kClawConnected && Constants.currentMode != Mode.SIM);
  }

  public boolean hasCoral() {
    // Assuming resistance can be inferred from the current draw
    double currentDraw = inputs.clawVel;
    return currentDraw < Constants.Manipulator.Claw.coralDetectionCurrentThreshold;

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
