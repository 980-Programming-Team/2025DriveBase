package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Funnel extends SubsystemBase {
  public FunnelIO io;
  public FunnelIOInputsAutoLogged inputs;

  // instead of has coeral check check if were in the range of a pose, robot

  private final Alert pivotMissingAlert;
  private final Alert intakeMissingAlert;

  private double setpoint;
  private FunnelStates state;

  private boolean requestIdle;
  private boolean requestFeed;
  public boolean requestClimb;

  private Timer feedTimer;

  public enum FunnelStates {
    IDLE,
    FEED // ,
    // CLIMB_READY
  }

  public Funnel(FunnelIO funnelIO) {
    this.io = funnelIO;

    inputs = new FunnelIOInputsAutoLogged();

    pivotMissingAlert = new Alert("Disconnected Pivot Motor", AlertType.kError);
    intakeMissingAlert = new Alert("Disconnected Intake Motor", AlertType.kError);

    setpoint = 0;
    state = FunnelStates.IDLE;
    requestIdle = true;
    requestFeed = false;
    requestClimb = false;

    feedTimer = new Timer();
    feedTimer.stop();
    feedTimer.reset();
  }

  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Funnel", inputs);

    // Logger.recordOutput("Funnel/Setpoint", setpoint);
    // Logger.recordOutput("Funnel/Position", getPosition());
    // Logger.recordOutput("Funnel/requestClimb", requestClimb);

    switch (state) {
      case IDLE:
        // io.setPosition(setpoint);
        io.set(0);

        if (requestFeed) {
          state = FunnelStates.FEED;
          // } else if (requestClimb) {
          //   state = FunnelStates.CLIMB_READY;
        }

        break;
      case FEED:
        io.set(Constants.Funnel.feedSpeed);

        if (feedTimer.get() <= 0) feedTimer.start();

        if (feedTimer.get() >= 0.25 || requestIdle) {
          state = FunnelStates.IDLE;
          feedTimer.stop();
          feedTimer.reset();
          requestIdle();
        }
        break;
        // case CLIMB_READY:
        //   if (inputs.pos <= setpoint) {
        //     io.setPivot(.9);
        //   } else {
        //     io.setPivot(0);
        //   }
        //   break;
    }

    // pivotMissingAlert.set(!inputs.kPivotConnected && Constants.currentMode != Mode.SIM);
    // intakeMissingAlert.set(!inputs.kIntakeConnected && Constants.currentMode != Mode.SIM);
  }

  public void requestFeed() {
    requestIdle = false;
    requestFeed = true;
  }

  public void requestIdle() {
    requestFeed = false;
    requestClimb = false;
    requestIdle = true;
  }

  // public void requestPosition(double position) {
  //   setpoint = position;

  //   if (setpoint > 5000) {
  //     requestIdle();

  //     requestClimb = true;
  //   }
  // }

  // public double getPosition() {
  //   return inputs.pos;
  // }

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
