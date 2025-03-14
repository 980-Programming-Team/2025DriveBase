package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LED.CANdleSystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Arm;
import frc.robot.subsystems.manipulator.Claw;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestPreScore;
  private boolean requestScore;
  private boolean requestClimbReady;
  private boolean requestDisable;

  private Superstates state;
  private Elevator elevator;
  private Arm arm;
  public Funnel funnel;
  private Claw claw;
  private Climber climber;

  public CANdleSystem candle;

  private Level level;
  // private Level prevLevel = Level.L2;

  private DigitalInput beamBreak;

  // private Timer feedingTimer;
  // private Timer pausedFeedingTimer;

  public static enum Superstates {
    IDLE,
    FEEDING,
    PRE_SCORE,
    SCOREL1,
    SCOREL2,
    SCORE,
    CLIMB_READY,
    DISABLED
  }

  public static enum Level {
    L1,
    L2,
    L3,
    L4
  }

  public Superstructure(
      Elevator elevator, Arm arm, Claw claw, Funnel funnel, Climber climber, CANdleSystem candle) {
    this.elevator = elevator;
    this.arm = arm;
    this.funnel = funnel;
    this.claw = claw;
    this.climber = climber;
    this.candle = candle;

    state = Superstates.IDLE;
    level = Level.L2;

    // feedingTimer = new Timer();
    // pausedFeedingTimer = new Timer();

    // feedingTimer.stop();
    // feedingTimer.reset();

    // pausedFeedingTimer.start();

    beamBreak = new DigitalInput(0);
  }

  @Override
  public void periodic() {

    // Logger.recordOutput("Superstructure/FeedingTimer", feedingTimer.get());
    // Logger.recordOutput("Superstructure/PausedFeedingTimer", pausedFeedingTimer.get());

    Logger.recordOutput("SuperStructure/BeamBreak", beamBreak.get());

    Logger.recordOutput("Superstructure/State", state.toString());
    Logger.recordOutput("Superstructure/Level", level.toString());
    switch (state) {
      case IDLE:
        elevator.requestHeight(-0.055);
        arm.requestPosition(0.025);
        claw.requestIdle();
        candle.SetLEDGreen();

        if (!funnel.requestClimb) {
          funnel.requestPosition(0.001);
        }

        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestClimbReady) {
          state = Superstates.CLIMB_READY;
        }
        break;
      case FEEDING:
        arm.requestPosition(-.08);
        elevator.requestHeight(-0.055);
        funnel.requestFeed();
        claw.requestFeed();

        candle.SetLEDYellow();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (level == Level.L1 && requestScore) {
          state = Superstates.SCOREL1;
        } else if (level == Level.L2 && requestScore) {
          state = Superstates.SCOREL2;
        } else if (requestScore) {
          state = Superstates.SCORE;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        }

        if (!beamBreak.get()) {
          state = Superstates.IDLE;
          unsetAllRequests();
        }

        break;
      case PRE_SCORE: // 13 inches away from reef for L2
        if (level == Level.L1) {
          arm.requestPosition(.005);
          elevator.requestHeight(-.052);
          candle.SetLEDOrange();
        } else if (level == Level.L2) {
          arm.requestPosition(0.12881910562515259);
          elevator.requestHeight(-0.147);
          candle.SetLEDRed();
        } else if (level == Level.L3) {
          arm.requestPosition(0.70);
          elevator.requestHeight(-0.10);
          candle.SetLEDGreen();
        } else if (level == Level.L4) {
          arm.requestPosition(0.72);
          elevator.requestHeight(-0.85);
          candle.SetLEDYellow();
        }
        claw.requestIdle();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (level == Level.L1 && requestScore) {
          state = Superstates.SCOREL1;
        } else if (level == Level.L2 && requestScore) {
          state = Superstates.SCOREL2;
        } else if (requestScore) {
          state = Superstates.SCORE;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        }

        break;
      case SCOREL1:
        claw.requestShootL1();

        if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestClimbReady) {
          state = Superstates.CLIMB_READY;
        } else if (requestIdle) {
          state = Superstates.IDLE;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestFeed) {
          state = Superstates.FEEDING;
        }
        break;
      case SCOREL2:
        claw.requestShootL2();

        if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestClimbReady) {
          state = Superstates.CLIMB_READY;
        } else if (requestIdle) {
          state = Superstates.IDLE;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestFeed) {
          state = Superstates.FEEDING;
        }

        break;
      case SCORE:
        claw.requestShoot();
        if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestClimbReady) {
          state = Superstates.CLIMB_READY;
        } else if (requestIdle) {
          state = Superstates.IDLE;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestFeed) {
          state = Superstates.FEEDING;
        }
        break;
      case CLIMB_READY:
        candle.SetLEDPurple();
        climber.requestPosition(120);
        break;
      case DISABLED:
        arm.enableCoastMode(true);
        break;
    }
  }

  public Superstates getState() {
    return state;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void requestPreScore() {
    unsetAllRequests();
    requestPreScore = true;
  }

  public void requestScore() {
    unsetAllRequests();
    requestScore = true;
  }

  public void requestClimbReady() {
    unsetAllRequests();
    requestClimbReady = true;
  }

  public void requestDisable() {
    unsetAllRequests();
    requestDisable = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestFeed = false;
    requestPreScore = false;
    requestScore = false;
    requestClimbReady = false;
    requestDisable = false;
  }

  public void requestLevel(Level level) {
    this.level = level;
  }

  public void requestLevel(int level) {
    switch (level) {
      case 1:
        this.level = Level.L1;
        break;

      case 2:
        this.level = Level.L2;
        break;

      case 3:
        this.level = Level.L3;
        break;

      case 4:
        this.level = Level.L4;
        break;
    }
  }

  // public boolean pieceSecured() {
  //   return claw.coralSecured();
  // }

  public void intakeCoral(Trigger action) {
    action.onTrue(
        new InstantCommand(
                () -> {
                  // ! Manual Override:
                  // pausedFeedingTimer.stop();
                  // pausedFeedingTimer.reset();

                  // if (feedingTimer.get() <= 0) feedingTimer.start(); end

                  if (elevator.getHeight() < .5) {
                    requestFeed();
                  }
                })
            .ignoringDisable(true));
    action.onFalse(
        new InstantCommand(
                () -> {
                  // ! Manual Override:
                  // feedingTimer.stop();
                  // feedingTimer.reset();

                  // if (pausedFeedingTimer.get() <= 0) pausedFeedingTimer.start(); end

                  funnel.requestIdle();
                  requestIdle();
                })
            .ignoringDisable(true));
  }
}
