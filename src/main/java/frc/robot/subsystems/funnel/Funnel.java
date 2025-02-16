package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import frc.robot.constants.Constants;

import org.jheaps.annotations.ConstantTime;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private FunnelIO io;
  private FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestDescore;

  private FunnelStates state = FunnelStates.STARTING_CONFIG;

  public enum FunnelStates {
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
    Logger.processInputs("Funnel", inputs);
    Logger.recordOutput("Funnel/State", state.toString());

    switch (state) {
      case STARTING_CONFIG:
        io.seedPivotPosition(inputs.pivotPosAbsMechanismRotations);
        state = FunnelStates.IDLE;
        break;
      case IDLE:
        io.setPivotPosition(Constants.Funnel.Pivot.stowedSetpointMechanismRotations);
        io.setIntakeVoltage(Constants.Funnel.Intake.);

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
