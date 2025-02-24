package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private FunnelIO io;
  private FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestClimb;

  private FunnelStates state = FunnelStates.STARTING_CONFIG;

  public enum FunnelStates {
    STARTING_CONFIG,
    IDLE,
    FEED,
    CLIMB_READY,
  }

  public Funnel(FunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);
    Logger.recordOutput("Funnel/State", state.toString());

    switch (state) {
      case STARTING_CONFIG:
        io.seedPivotPosition(inputs.pivotPosAbsMechanismRotations);
        state = FunnelStates.IDLE;
        break;
      case IDLE:
        io.setPivotPosition(Constants.Funnel.Pivot.stowedSetpointMechanismRotations);
        io.setIntakeVoltage(0);

        if (requestFeed) {
          state = FunnelStates.FEED;
        }

        if (requestClimb) {
          state = FunnelStates.CLIMB_READY;
        }

        break;
      case FEED:
        io.setIntakeVoltage(Constants.Funnel.Intake.feedVoltage);
        if (requestIdle) {
          state = FunnelStates.IDLE;
        }
        break;
      case CLIMB_READY:
        io.setPivotPosition(Constants.Funnel.Pivot.climbReadySetpointMechanismRotations);
        if (requestIdle) {
          state = FunnelStates.IDLE;
        }
        break;
    }
  }

  public boolean atClimbSetpoint() {
    return Util.atReference(
        inputs.pivotPosAbsMechanismRotations,
        Constants.Funnel.Pivot.climbReadySetpointMechanismRotations,
        Constants.Funnel.Pivot.setpointToleranceMechanismRotations,
        true);
  }

  public double getPivotPosition() {
    return inputs.pivotPosAbsMechanismRotations;
  }

  // Use method only to reset state when robot is disabled
  public void forceIdle() {
    unsetAllRequests();
    state = FunnelStates.IDLE;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void requestClimb() {
    unsetAllRequests();
    requestClimb = true;
  }

  private void unsetAllRequests() {
    requestFeed = false;
    requestIdle = false;
    requestClimb = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
