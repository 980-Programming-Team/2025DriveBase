package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs;

  private final Alert armMissingAlert;

  // private boolean requestIdle;
  // private boolean requestFeed;
  // private boolean requestL2;
  // private boolean requestL3;
  // private boolean requestL4;

  private double setpoint;

  private ArmStates state = ArmStates.STARTING_CONFIG;

  public enum ArmStates {
    STARTING_CONFIG,
    REQUEST_SETPOINT
  }

  public Arm(ArmIO io) {
    this.io = io;

    setpoint = 0.0;
    inputs = new ArmIOInputsAutoLogged();
    armMissingAlert = new Alert("Disconnected Arm Motor", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput("Manipulator/State", state.toString());
    Logger.recordOutput("Manipulator/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = ArmStates.REQUEST_SETPOINT;
        }
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

  public double getPosition() {
    return inputs.pos;
  }

  public double getVelocity() {
    return inputs.velMetersPerSecond;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void enableCoastMode(boolean enable) {
    io.enableCoastMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
