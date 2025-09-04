package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;

public class Claw extends SubsystemBase {
  public ClawIO io;
  // private ClawIOInputsAutoLogged inputs;

  private final Alert clawMissingAlert;

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestAutoFeed;
  private boolean requestShoot;
  private boolean requestShootL1;
  private boolean requestShootL2;
  private boolean requestShootL4;

  // private boolean coralSecured;
  private ClawStates state;

  private Timer shootTimer;

  public enum ClawStates {
    IDLE,
    FEED,
    AUTO_FEED,
    SHOOTL1,
    SHOOTL2,
    SHOOT,
    SHOOTL4
  }

  public Claw(ClawIO io) {
    this.io = io;

    // inputs = new ClawIOInputsAutoLogged();
    clawMissingAlert = new Alert("Disconnected Claw Motor", AlertType.kError);
    state = ClawStates.IDLE;
    shootTimer = new Timer();
    shootTimer.stop();
    shootTimer.reset();
  }

  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Manipulator", inputs);
    // Logger.recordOutput("Manipulator/Coral Detection", hasCoral());
    // Logger.recordOutput("Manipulator/Timer", shootTimer.get());
    // Logger.recordOutput("Manipulator/ClawState", state.toString());

    switch (state) {
      case IDLE:
        // how to get a button press to do an action without default command:
        if (RobotContainer.driver.getDriver().povUp().getAsBoolean()) {
          io.setClawSpeed(-Constants.Manipulator.Claw.scoreSpeed / 4);
        } else if (RobotContainer.driver.getDriver().povDown().getAsBoolean()) {
          io.setClawSpeed(Constants.Manipulator.Claw.scoreSpeed / 4);
        } else {
          // otherwise return to default state
          io.stop();
        }

        if (requestFeed && shootTimer.get() <= 0) {
          state = ClawStates.FEED;
        } else if (requestAutoFeed && shootTimer.get() <= 0) {
          state = ClawStates.AUTO_FEED;
        } else if (requestShoot && shootTimer.get() <= 0) {
          state = ClawStates.SHOOT;
        } else if (requestShootL1 && shootTimer.get() <= 0) {
          state = ClawStates.SHOOTL1;
        } else if (requestShootL2 && shootTimer.get() <= 0) {
          state = ClawStates.SHOOTL2;
        } else if (requestShootL4 && shootTimer.get() <= 0) {
          state = ClawStates.SHOOTL4;
        }
        break;
      case FEED:
        io.setClawSpeed(Constants.Manipulator.Claw.feedSpeed);

        if (shootTimer.get() <= 0) shootTimer.start();

        if (shootTimer.get() >= 3 || requestIdle) {
          // uses timeout so motors don't run infinitely or on toggle when inaccessible
          state = ClawStates.IDLE;
          shootTimer.stop();
          shootTimer.reset();
          requestIdle();
        }
        break;
      case AUTO_FEED:
        io.setClawSpeed(Constants.Manipulator.Claw.feedSpeed);

        if (shootTimer.get() <= 0) shootTimer.start();

        if (shootTimer.get() >= 3 || requestIdle) {
          state = ClawStates.IDLE;
          shootTimer.stop();
          shootTimer.reset();
          requestIdle();
        }
        break;
      case SHOOTL1:
        io.setClawSpeed(Constants.Manipulator.Claw.scoreL1Speed);

        if (shootTimer.get() <= 0) shootTimer.start();

        if (shootTimer.get() >= 2 || requestIdle) {
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
      case SHOOTL4:
        io.setClawSpeed(Constants.Manipulator.Claw.scoreL4Speed);

        if (shootTimer.get() <= 0) shootTimer.start();

        if (shootTimer.get() >= 0.25 || requestIdle) {
          state = ClawStates.IDLE;

          Superstructure.arm.requestPosition(4.33);
          shootTimer.stop();
          shootTimer.reset();
          requestIdle();
        }
        break;
    }

    // clawMissingAlert.set(!inputs.kClawConnected && Constants.currentMode != Mode.SIM);
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

  public void requestAutoFeed() {
    unsetAllRequests();
    requestAutoFeed = true;
  }

  public void requestShoot() {
    unsetAllRequests();
    requestShoot = true;
  }

  public void requestShootL1() {
    unsetAllRequests();
    requestShootL1 = true;
  }

  public void requestShootL2() {
    unsetAllRequests();
    requestShootL2 = true;
  }

  public void requestShootL4() {
    unsetAllRequests();
    requestShootL4 = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestFeed = false;
    requestShootL2 = false;
    requestShoot = false;
    requestShootL1 = false;
    requestShootL4 = false;
    requestAutoFeed = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
