package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestEject;
  private boolean requestPreScore;
  private boolean requestPreScoreFlip;
  private boolean requestScore;

  private Superstates state = Superstates.IDLE;
  private Elevator elevator;
  private Manipulator arm;
  private Funnel funnel;

  private Level level = Level.L2;
  private Level prevLevel = Level.L2;

  public static enum Superstates {
    IDLE,
    FEEDING,
    L2,
    L3,
    L4,
    SCORE
  }

  public static enum Level {
    L2,
    L3,
    L4
  }

  public Superstructure(Elevator elevator, Manipulator arm, Funnel funnel) {
    this.elevator = elevator;
    this.arm = arm;
    this.funnel = funnel;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", state.toString());
    switch (state) {
      case IDLE:
        elevator.requestHeight(0);
        arm.requestIdle();
        funnel.requestIdle();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (requestFeed && !arm.hasCoral() && elevator.atSetpoint()) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestPreScoreFlip) {
          state = Superstates.SAFE_FLIP;
        }
        break;
      case FEEDING:
      arm.requestFeed();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (arm.hasCoral()) {
          if (arm.coralSecured()) {
            state = Superstates.IDLE;
            unsetAllRequests(); // account for automation from sensor triggers
          }
        } else if (requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case EJECT:
      arm.requestEject();

        if (requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case PRE_SCORE:
        if (level == Level.L2) {
          elevator.requestHeight(0);
        } else if (level == Level.L2) {
          elevator.requestHeight(Constants.Scoring.L2ScoringHeight);
        } else if (level == Level.L3) {
          elevator.requestHeight(Constants.Scoring.L3ScoringHeight);
        }
        funnel.requestIdle();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (requestPreScoreFlip) {
          state = Superstates.SAFE_FLIP;
        } else if (requestScore && elevator.atSetpoint() && arm.coralSecured()) {
          state = Superstates.SCORE;
        }
        break;
      case SAFE_FLIP:
        if (level == Level.L2) {
          elevator.requestHeight(0);
          prevLevel = level;
        } else if (level == Level.L2) {
          elevator.requestHeight(Constants.Scoring.L2SafeFlipHeight);
          prevLevel = level;
        } else if (level == Level.L3) {
          elevator.requestHeight(Constants.Scoring.L3ScoringHeight);
          prevLevel = level;
        }

        if (elevator.atSetpoint() && level != Level.L2) {
          funnel.requestDescore();
        }

        if (funnel.atDeploySetpoint() || level == Level.L2) {
          state = Superstates.PRE_SCORE_FLIP;
        }
        break;
      case PRE_SCORE_FLIP:
        if (prevLevel != level) {
          state = Superstates.TRANSITION_FLIP;
          break;
        }
        if (level == Level.L2) {
          elevator.requestHeight(0);
          prevLevel = level;
        } else if (level == Level.L2) {
          elevator.requestHeight(Constants.Scoring.L2ScoringHeight);
          prevLevel = level;
        } else if (level == Level.L3) {
          elevator.requestHeight(Constants.Scoring.L3ScoringHeight);
          prevLevel = level;
        }

        if (requestIdle) {
          state = Superstates.SAFE_RETRACT;
        } else if (requestPreScore) {
          state = Superstates.SAFE_RETRACT;
        } else if (requestScore && elevator.atSetpoint() && arm.coralSecured()) {
          state = Superstates.SCORE;
        }
        break;
      case TRANSITION_FLIP:
        if (prevLevel == Level.L2) {
          elevator.requestHeight(0);
        } else if (prevLevel == Level.L2) {
          elevator.requestHeight(Constants.Scoring.L2SafeFlipHeight);
        } else if (level == Level.L3) {
          elevator.requestHeight(Constants.Scoring.L3ScoringHeight);
        }

        if (elevator.atSetpoint()) {
          funnel.requestIdle();
        }

        if (funnel.getPivotPosition() < Constants.Scoring.safeFlipPosition) {
          state = Superstates.SAFE_FLIP;
        }
        break;
      case SAFE_RETRACT:
        if (level == Level.L2) {
          elevator.requestHeight(0);
        } else if (level == Level.L2) {
          elevator.requestHeight(Constants.Scoring.L2SafeFlipHeight);
        } else if (level == Level.L3) {
          elevator.requestHeight(Constants.Scoring.L3ScoringHeight);
        }

        if (elevator.atSetpoint()) {
          funnel.requestIdle();
        }

        if (funnel.getPivotPosition() < Constants.Scoring.safeFlipPosition) {
          if (requestIdle) {
            state = Superstates.IDLE;
          } else if (requestPreScore) {
            state = Superstates.PRE_SCORE;
          }
        }
        break;
      case SCORE:
      arm.requestShoot();

        if (!arm.coralSecured() && requestIdle) {
          state = Superstates.SAFE_RETRACT;
        }
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

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  public void requestPreScore() {
    unsetAllRequests();
    requestPreScore = true;
  }

  public void requestPreScoreFlip() {
    unsetAllRequests();
    requestPreScoreFlip = true;
  }

  public void requestScore() {
    unsetAllRequests();
    requestScore = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestEject = false;
    requestFeed = false;
    requestPreScore = false;
    requestPreScoreFlip = false;
    requestScore = false;
  }

  public void requestLevel(Level level) {
    this.level = level;
  }

  public boolean pieceSecured() {
    return arm.coralSecured();
  }
}