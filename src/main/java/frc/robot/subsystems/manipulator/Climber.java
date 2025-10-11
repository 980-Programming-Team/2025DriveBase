package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private double setpoint;
  private ClimberState state;

  private ClimberIOInputsAutoLogged inputs;

  private enum ClimberState {
    IDLE,
    EXTENDING,
    RETRACTING,
  }

  public Climber(ClimberIO io) {
    this.io = io;
    this.state = ClimberState.IDLE;
    setpoint = 0.0;
    io.zeroPosition();
    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    runClimberStateMachine();
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/State", this.state);
    Logger.recordOutput("Climber/Position", inputs.climberMotorPos);
  }

  public void extendClimber() {
    state = ClimberState.EXTENDING;
  }

  public void retractClimber() {
    state = ClimberState.RETRACTING;
  }

  public void stop() {
    this.state = ClimberState.IDLE;
  }

  private void runClimberStateMachine() {
    if (state == ClimberState.IDLE) {
      io.setClimberVoltage(0.0);
    } else if (state == ClimberState.EXTENDING) {
      if (inputs.climberMotorPos >= ClimberConstants.EXTENDED_POSITION_ROT) {
        state = ClimberState.IDLE;
      } else {
        io.setClimberVoltage(ClimberConstants.EXTENDING_VOLTAGE);
      }
    } else if (state == ClimberState.RETRACTING) {

      if (inputs.climberMotorPos <= ClimberConstants.RETRACTED_POSITION_ROT) {
        state = ClimberState.IDLE;
      } else {
        io.setClimberVoltage(ClimberConstants.RETRACTING_VOLTAGE);
      }
    }
  }
}
