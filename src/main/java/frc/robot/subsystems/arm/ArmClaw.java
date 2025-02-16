package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmClawIO.ArmClawIOInputs;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public class ArmClaw extends SubsystemBase {
  private ArmClawIO io;
  private ArmClawIOInputs inputs;

  private boolean OURCODE = false; //!

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestL2;
  private boolean requestL3;
  private boolean requestL4;
  private boolean requestEject;
  private boolean requestShoot;

  private boolean coralSecured;
  private ArmStates state = ArmStates.IDLE;

  private Timer shootTimer = new Timer();

  public enum ArmStates {
    IDLE,
    FEED,
    L2,
    L3,
    L4,
    EJECT,
    SHOOT,
    SECURING_CORAL
  }

  public ArmClaw(ArmClawIO io) {
    this.io = io;

    inputs = new ArmClawIOInputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ArmClaw", null); //! 
    Logger.recordOutput("ArmClaw/State", state.toString());
    Logger.recordOutput("ArmClaw/Coral Detection", hasCoral());
    Logger.recordOutput("ArmClaw/Coral Detection", coralSecured());

    switch (state) {
      case IDLE:
        io.stop();

        // reset coral secured in cases where coral is removed manually from robot
        if (!hasCoral()) {
          coralSecured = false;
        }

        if (requestEject && coralSecured()) {
          state = ArmStates.EJECT;
        } else if (requestFeed && !coralSecured()) {
          state = ArmStates.FEED;
        } else if (requestShoot && coralSecured) {
          state = ArmStates.SHOOT;
        }
        break;
      case FEED:
        io.setArmVoltage(Constants.Arm.armFeedVoltage);
        io.setClawVoltage(Constants.Arm.clawFeedVoltage);

        if (requestEject && coralSecured()) {
          state = ArmStates.EJECT;
        } else if (OURCODE) { // else if (coral intake successful) //! inputs.frontBeamBreakTriggered
          state = ArmStates.SECURING_CORAL;
        } else if (requestIdle) {
          state = ArmStates.IDLE;
        }
        break;
      case SECURING_CORAL:
        io.setArmVoltage(Constants.Arm.secondFeedVoltage);

        if (requestEject) {
          state = ArmStates.EJECT;
        } else if (OURCODE) { // else if (coral ready to eject) //! !inputs.backBeamBreakTriggered && inputs.frontBeamBreakTriggered
          state = ArmStates.IDLE;
          coralSecured = true;
          unsetAllRequests(); // account for automation from sensor triggers
        }
        break;
      case SHOOT:
        io.setArmVoltage(Constants.Arm.shootVoltage);

        if (requestEject) {
          state = ArmStates.EJECT;
        } else if (OURCODE) { // else if (coral eject successful) //! !inputs.frontBeamBreakTriggered
          shootTimer.start();
          if (shootTimer.hasElapsed(Constants.Arm.shootWaitTimerSec)) {
            coralSecured = false;
            shootTimer.stop();
            shootTimer.reset();
            state = ArmStates.IDLE;
            unsetAllRequests(); // account for automation from sensor triggers
          }
        }
        break;
      case EJECT:
        io.setArmVoltage(Constants.Arm.spitVoltage);
        coralSecured = false;
        if (OURCODE) { // else if (coral passed intake, not ready to eject) //! inputs.backBeamBreakTriggered
          state = ArmStates.FEED;
        }
        break;
    }
  }

  public boolean hasCoral() {
    // Assuming resistance can be inferred from the current draw
    double currentDraw = inputs.clawStatorCurrentAmps;
    return currentDraw > Constants.Arm.coralDetectionCurrentThreshold;

    //    return inputs.frontBeamBreakTriggered || inputs.backBeamBreakTriggered;
  }

  public boolean coralSecured() {
    return coralSecured;
  }

  // Use method only to reset state when robot is disabled
  public void forceIdle() {
    unsetAllRequests();
    state = ArmStates.IDLE;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void reqquestL2() {
    unsetAllRequests();
    requestL2 = true;
  }

  public void requestL3() {
    unsetAllRequests();
    requestL3 = true;
  }

  public void requestL4() {
    unsetAllRequests();
    requestL4 = true;
  }

  public void requestShoot() {
    unsetAllRequests();
    requestShoot = true;
  }

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  private void unsetAllRequests() {
    requestFeed = false;
    requestIdle = false;
    requestShoot = false;
    requestEject = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
