package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.LED.CANdleSystem;
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
  private Funnel funnel;
  private Claw claw;
  private CANdleSystem candle;

  private Level level;
  // private Level prevLevel = Level.L2;

  private DigitalInput beamBreak;

  public static enum Superstates {
    IDLE,
    FEEDING,
    PRE_SCORE,
    SCOREL2,
    SCORE,
    CLIMB_READY,
    DISABLED
  }

  public static enum Level {
    L2,
    L3,
    L4
  }

  public Superstructure(Elevator elevator, Arm arm, Claw claw, Funnel funnel, CANdleSystem candle) {
    this.elevator = elevator;
    this.arm = arm;
    this.funnel = funnel;
    this.claw = claw;
    this.candle = candle;

    state = Superstates.IDLE;
    level = Level.L2;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Superstructure/State", state.toString());
    Logger.recordOutput("Superstructure/Level", level.toString());
    switch (state) {
      case IDLE:
        elevator.requestHeight(0.001);
        arm.requestPosition(0.001);
        funnel.requestPosition(0.001);
        claw.requestIdle();
        candle.SetLEDGreen();

        //   if (requestFeed /*&& !claw.hasCoral() && elevator.atSetpoint()*/) {
        //     state = Superstates.FEEDING;
        //   } else if (requestPreScore) {
        //     state = Superstates.PRE_SCORE;
        //   } else if (requestClimbReady) {
        //     state = Superstates.CLIMB_READY;
        //   }
        //   break;
        // case FEEDING:
        //   elevator.requestHeight(0);
        //   // arm.requestPosition(-.0175);
        //   funnel.requestFeed();
        //   claw.requestFeed();

        // if (claw.hasCoral()) {
        //   if (claw.coralSecured()) {
        //     state = Superstates.IDLE;
        //     unsetAllRequests(); // account for automation from sensor triggers
        //   }
        // } else if (requestIdle) {
        //   state = Superstates.IDLE;
        // }
        break;
      case PRE_SCORE: // 13 inches away from reef for L2
        if (level == Level.L2) {
          arm.requestPosition(0.12881910562515259);
          elevator.requestHeight(-0.147);
          candle.SetLEDRed();
        } else if (level == Level.L3) {
          arm.requestPosition(0.80);
          elevator.requestHeight(-0.10);
          candle.SetLEDGreen();
        } else if (level == Level.L4) {
          arm.requestPosition(0.75);
          elevator.requestHeight(-0.85);
          candle.SetLEDYellow();
        }
        claw.requestIdle();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (level == Level.L2
            && (requestScore /*&& elevator.atSetpoint()*/ /*&& claw.coralSecured()*/)) {
          state = Superstates.SCOREL2;
        } else if (requestScore /*&& elevator.atSetpoint()*/ /*&& claw.coralSecured()*/) {
          state = Superstates.SCORE;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
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
        // if (
        // /*!claw.coralSecured() &&*/ requestIdle) {
        //   state = Superstates.IDLE;
        // }
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
        // if (
        // /*!claw.coralSecured() &&*/ requestIdle) {
        //   state = Superstates.IDLE;
        // }
        break;
      case CLIMB_READY:
        funnel.requestPosition(0.3);
        break;
      case DISABLED:
        arm.enableCoastMode(true);
        break;
      default:
        claw.io.setClawSpeed(0);
        funnel.io.set(0);
        unsetAllRequests();
        requestIdle();
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

  public boolean pieceSecured() {
    return claw.coralSecured();
  }

  // manual override
  public void intakeCoral() {
    elevator.requestHeight(0);
    // // arm.requestPosition(-.0175);
    // funnel.requestFeed();
    // claw.requestFeed();

    unsetAllRequests();
    state = Superstates.FEEDING;

    funnel.io.set(Constants.Funnel.feedSpeed);
    claw.io.setClawSpeed(Constants.Manipulator.Claw.feedSpeed);
  }
}
