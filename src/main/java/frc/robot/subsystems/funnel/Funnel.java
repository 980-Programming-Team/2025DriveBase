package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private FunnelIO io;
  private FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestDescore;

  private FlipperStates state = FlipperStates.STARTING_CONFIG;

  public enum FlipperStates {
    STARTING_CONFIG,
    IDLE,
    CLIMB_READY,
  }

  public Funnel(FunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flipper", inputs);
    Logger.recordOutput("Flipper/State", state.toString());

    switch (state) {
      case SEED_POSITION:
        io.seedPivotPosition(inputs.pivotPosAbsMechanismRotations);
        state = FlipperStates.IDLE;
        break;
      case IDLE:
        io.setPivotPosition(Constants.Funnel.Pivot.stowedSetpointMechanismRotations);
        io.setRollerVoltage(0);

        if (requestDescore) {
          state = FlipperStates.FLIP;
        }
        break;
      case FLIP:
        io.setPivotPosition(Constants.Funnel.Pivot.deployedSetpointMechanismRotations);
        if (requestIdle) {
          state = FlipperStates.IDLE;
        }
        if (atDeploySetpoint()) {
          state = FlipperStates.SPIN;
        }
        break;
      case SPIN:
        io.setRollerVoltage(Constants.Funnel.Roller.descoreVoltage);
        if (requestIdle) {
          state = FlipperStates.IDLE;
        }
        break;
    }
  }

  public boolean atDeploySetpoint() {
    return Util.atReference(
        inputs.pivotPosAbsMechanismRotations,
        Constants.Funnel.Pivot.deployedSetpointMechanismRotations,
        Constants.Funnel.Pivot.setpointToleranceMechanismRotations,
        true);
  }

  public double getPivotPosition() {
    return inputs.pivotPosAbsMechanismRotations;
  }

  // Use method only to reset state when robot is disabled
  public void forceIdle() {
    unsetAllRequests();
    state = FlipperStates.IDLE;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestDescore() {
    unsetAllRequests();
    requestDescore = true;
  }

  private void unsetAllRequests() {
    requestDescore = false;
    requestIdle = false;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
