package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private Superstates state = Superstates.IDLE;
  private Elevator elevator;
  private Arm arm;
  private Funnel funnel;
  private Claw claw;
  private Climber climber;

  private Level level = Level.L2;
  private Level prevLevel = Level.L2;

  public static enum Superstates {
    IDLE,
    FEEDING,
    PRE_SCORE,
    SCOREL2,
    SCORE
  }

  public static enum Level {
    L2,
    L3,
    L4
  }

  public Superstructure(Elevator elevator, Arm arm, Claw claw, Funnel funnel, Climber climber) {
    this.elevator = elevator;
    this.arm = arm;
    this.funnel = funnel;
    this.claw = claw;
    this.climber = climber;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", state.toString());
    switch (state) {
      case IDLE:
        elevator.requestHeight(0);
        arm.requestIdle();
        funnel.requestIdle();
        claw.requestIdle();
        climber.requestIdle(0);

        if (requestFeed && !claw.hasCoral() && elevator.atSetpoint()) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        }
        break;
      case FEEDING:
        elevator.requestHeight(0);
        arm.requestFeed();
        claw.requestFeed();

        if (claw.hasCoral()) {
          if (claw.coralSecured()) {
            state = Superstates.IDLE;
            unsetAllRequests(); // account for automation from sensor triggers
          }
        } else if (requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case PRE_SCORE:
        if (level == Level.L2) {
          arm.requestL2();
          elevator.requestHeight(0.14);
        } else if (level == Level.L3) {
          arm.requestL3();
          elevator.requestHeight(0.10);
        } else if (level == Level.L4) {
          arm.requestL4();
          elevator.requestHeight(0.71);
        }
        funnel.requestIdle();
        climber.requestIdle(0);
        claw.requestIdle();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (level == Level.L2
            && (requestScore && elevator.atSetpoint() && claw.coralSecured())) {
          state = Superstates.SCOREL2;
        } else if (requestScore && elevator.atSetpoint() && claw.coralSecured()) {
          state = Superstates.SCORE;
        }
        break;
      case SCOREL2:
        claw.requestShootL2();

        if (!claw.coralSecured() && requestIdle) {
          state = Superstates.IDLE;
        }
        break;
      case SCORE:
        claw.requestShoot();

        if (!claw.coralSecured() && requestIdle) {
          state = Superstates.IDLE;
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

  public void requestPreScore() {
    unsetAllRequests();
    requestPreScore = true;
  }

  public void requestScore() {
    unsetAllRequests();
    requestScore = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestFeed = false;
    requestPreScore = false;
    requestScore = false;
  }

  public void requestLevel(Level level) {
    this.level = level;
  }

  public boolean pieceSecured() {
    return claw.coralSecured();
  }
}
