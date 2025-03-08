package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public ElevatorIO io;
  public ElevatorIOInputsAutoLogged inputs;

  private final Alert leaderMissingAlert;
  private final Alert followerMissingAlert;

  private double setpoint;
  private ElevatorStates state;

  public enum ElevatorStates {
    STARTING_CONFIG,
    REQUEST_SETPOINT
  }

  public Elevator(ElevatorIO elevatorIO) {
    this.io = elevatorIO;

    inputs = new ElevatorIOInputsAutoLogged();

    leaderMissingAlert = new Alert("Disconnected RoboRIO Elevator Motor", AlertType.kError);
    followerMissingAlert = new Alert("Disconnected PDH Elevator Motor", AlertType.kError);

    setpoint = 0;
    state = ElevatorStates.STARTING_CONFIG;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/State", state.toString());
    Logger.recordOutput("Elevator/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = ElevatorStates.REQUEST_SETPOINT;
        }
        break;
      case REQUEST_SETPOINT:
        if (setpoint != 0.0) {
          io.setHeight(setpoint);
        }
        break;
    }

    leaderMissingAlert.set(!inputs.kRoborioMotorConnected && Constants.currentMode != Mode.SIM);
    followerMissingAlert.set(!inputs.kPDHMotorConnected && Constants.currentMode != Mode.SIM);
  }

  public void requestHeight(double heightMeters) {
    setpoint = heightMeters;
  }

  public double getHeight() {
    return inputs.posMeters;
  }

  public double getVelocity() {
    return inputs.velMetersPerSecond;
  }

  public boolean atSetpoint() {
    return Util.atReference(
        inputs.posMeters, setpoint, Constants.Elevator.setpointToleranceMeters, true);
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
