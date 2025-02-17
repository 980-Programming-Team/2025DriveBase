package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public ClimberIO io;
  public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();


  private final Alert leaderMissingAlert = new Alert("Disconnected Near L1 Mechanism Climber Motor", AlertType.kError);
  private final Alert followerMissingAlert = new Alert("Disconnected Near Funnel Climber Motor", AlertType.kError);

  private double setpoint = 0;
  private ClimberStates state = ClimberStates.STARTING_CONFIG;

  private Timer homingTimer = new Timer();

  public enum ClimberStates {
    STARTING_CONFIG,
    HOMING,
    REQUEST_SETPOINT
  }

  public Climber(ClimberIO climberIO) {
    this.io = climberIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = ClimberStates.HOMING;
        }
        break;
      case HOMING:
        homingTimer.start();
        io.setArmVoltage(Constants.Elevator.homingVoltage);
        if (homingTimer.hasElapsed(Constants.Elevator.homingThresholdSec)
            && Math.abs(inputs.velMetersPerSecond) < Constants.Elevator.homingVelocityThreshold) {
          io.setVoltage(0);
          io.seedPosition(0);
          homingTimer.stop();
          homingTimer.reset();
          state = ElevatorStates.REQUEST_SETPOINT;
        }
        break;
      case REQUEST_SETPOINT:
        io.setHeight(setpoint);
        break;
    }

   leaderMissingAlert.set(!inputs.kNearL1Connected && Constants.currentMode != Mode.SIM);
   followerMissingAlert.set(!inputs.kNearFunnelConnected && Constants.currentMode != Mode.SIM);
    
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

  public void setHomingState(boolean isHomed) {
    state = isHomed ? ElevatorStates.REQUEST_SETPOINT : ElevatorStates.HOMING;
  }

  public void seedPosition(double motorRotations) {
    io.seedPosition(motorRotations);
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
