package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Alert armMissingAlert = new Alert("Disconnected Arm Motor", AlertType.kError);

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestL2;
  private boolean requestL3;
  private boolean requestL4;

  private Claw claw;

  private boolean coralSecured;
  private ArmStates state = ArmStates.IDLE;

  private Timer shootTimer = new Timer();
  private Timer homingTimer = new Timer();

  public enum ArmStates {
    IDLE,
    FEED,
    L2,
    L3,
    L4
  }

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput("Manipulator/State", state.toString());

    switch (state) {
      case IDLE:
        io.stop();
        io.setArmPosition(Constants.Manipulator.Arm.stowedSetpointMechanismRotations);

        if (requestFeed) {
          state = ArmStates.FEED;
        } else if (requestL2) {
          state = ArmStates.L2;
        } else if (requestL3) {
          state = ArmStates.L3;
        } else if (requestL4) {
          state = ArmStates.L4;
        }
        break;
      case FEED:
        io.setArmPosition(Constants.Manipulator.Arm.feedSetpointMechanismRotations);

        if (requestL2) {
          state = ArmStates.L2;
        } else if (requestL3) {
          state = ArmStates.L3;
        } else if (requestL4) {
          state = ArmStates.L4;
        } else if (requestIdle) {
          state = ArmStates.IDLE;
        }
        break;
      case L2:
        io.setArmPosition(Constants.Manipulator.Arm.l2SetpointMechanismRotations);

        if (requestFeed) {
          state = ArmStates.FEED;
        } else if (requestL3) {
          state = ArmStates.L3;
        } else if (requestL4) {
          state = ArmStates.L4;
        } else if (requestIdle) {
          state = ArmStates.IDLE;
        }
        break;
      case L3:
        io.setArmPosition(Constants.Manipulator.Arm.l3SetpointMechanismRotations);

        if (requestFeed) {
          state = ArmStates.L2;
        } else if (requestFeed) {
          state = ArmStates.FEED;
        } else if (requestL4) {
          state = ArmStates.L4;
        } else if (requestIdle) {
          state = ArmStates.IDLE;
        }
        break;
      case L4:
        if (requestFeed) {
          state = ArmStates.FEED;
        } else if (requestL2) {
          state = ArmStates.L2;
        } else if (requestL3) {
          state = ArmStates.L3;
        } else if (requestIdle) {
          state = ArmStates.IDLE;
        }
        break;
    }

    armMissingAlert.set(!inputs.kArmConnected && Constants.currentMode != Mode.SIM);
  }

  public boolean atFeedSetpoint() {
    return Util.atReference(
        inputs.armPosAbsMechanismRotations,
        Constants.Manipulator.Arm.feedSetpointMechanismRotations,
        Constants.Manipulator.Arm.setpointToleranceMechanismRotations,
        true);
  }

  public boolean atL2Setpoint() {
    return Util.atReference(
        inputs.armPosAbsMechanismRotations,
        Constants.Manipulator.Arm.l2SetpointMechanismRotations,
        Constants.Manipulator.Arm.setpointToleranceMechanismRotations,
        true);
  }

  public boolean atL3Setpoint() {
    return Util.atReference(
        inputs.armPosAbsMechanismRotations,
        Constants.Manipulator.Arm.l3SetpointMechanismRotations,
        Constants.Manipulator.Arm.setpointToleranceMechanismRotations,
        true);
  }

  public boolean atL4Setpoint() {
    return Util.atReference(
        inputs.armPosAbsMechanismRotations,
        Constants.Manipulator.Arm.l4SetpointMechanismRotations,
        Constants.Manipulator.Arm.setpointToleranceMechanismRotations,
        true);
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

  public void requestL2() {
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

  private void unsetAllRequests() {
    requestIdle = false;
    requestFeed = false;
    requestL2 = false;
    requestL3 = false;
    requestL4 = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
