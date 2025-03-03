package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs;

  private final Alert armMissingAlert;

  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestL2;
  private boolean requestL3;
  private boolean requestL4;

  private double setpoint;

  // private Claw claw;

  // private boolean coralSecured;

  private ArmStates state = ArmStates.STARTING_CONFIG;

  // private Timer shootTimer;
  // private Timer homingTimer;

  public enum ArmStates {
    STARTING_CONFIG,
    HOMING,
    REQUEST_SETPOINT
  }

  public Arm(ArmIO io) {
    this.io = io;

    setpoint = 0.0;
    inputs = new ArmIOInputsAutoLogged();
    armMissingAlert = new Alert("Disconnected Arm Motor", AlertType.kError);

    // shootTimer = new Timer();
    // homingTimer = new Timer();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput("Manipulator/State", state.toString());

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = ArmStates.HOMING;
        }
        break;
      case HOMING:
        // homingTimer.start();
        // io.setVoltage(Constants.Elevator.homingVoltage);
        // if (homingTimer.hasElapsed(Constants.Elevator.homingThresholdSec)
        // && Math.abs(inputs.velMetersPerSecond) < Constants.Elevator.homingVelocityThreshold) {
        // io.setVoltage(0);
        // io.seedPosition(0);
        // homingTimer.stop();
        // homingTimer.reset();
        state = ArmStates.REQUEST_SETPOINT;
        // }
        break;
      case REQUEST_SETPOINT:
        if (setpoint != 0.0) {
          io.setArmPosition(setpoint);
        }
        break;
    }

    armMissingAlert.set(!inputs.kArmConnected && Constants.currentMode != Mode.SIM);
  }

  public void requestPosition(double position) {
    setpoint = position;
  }

  public double getHeight() {
    return inputs.armPosMotorRotations;
  }

  public double getVelocity() {
    return inputs.armVelocity;
  }

  public boolean atSetpoint() {
    return Util.atReference(
        inputs.armPosMotorRotations, setpoint, Constants.Elevator.setpointToleranceMeters, true);
  }

  public void setHomingState(boolean isHomed) {
    state = isHomed ? ArmStates.REQUEST_SETPOINT : ArmStates.HOMING;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
