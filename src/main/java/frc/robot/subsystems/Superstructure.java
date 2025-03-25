package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LED.CANdleSystem;
// import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Arm;
import frc.robot.subsystems.manipulator.Claw;

public class Superstructure extends SubsystemBase {
  private boolean requestIdle;
  private boolean requestPreFeed;
  private boolean requestFeed;
  private boolean requestPreScore;
  private boolean requestScore;
  // private boolean requestClimbReady;
  private boolean requestDisable;

  private Superstates state;
  private Elevator elevator;
  private Arm arm;
  public Funnel funnel;
  private Claw claw;
  // private Climber climber;

  public CANdleSystem candle;

  private Level level;

  private LaserCan ohtaniLaser;

  // pretty sure that both encoders turn counterclockwise
  // if negative turning for positive movement, use setInverted()
  private DutyCycleEncoder elevatorEncoder;
  private DutyCycleEncoder armEncoder;

  public static enum Superstates {
    IDLE,
    PRE_FEED,
    FEEDING,
    PRE_SCORE,
    SCOREL1,
    SCOREL2,
    SCORE,
    SCOREL4,
    // CLIMB_READY,
    DISABLED
  }

  public static enum Level {
    L1,
    L2,
    L3,
    L4
  }

  public Superstructure(
      Elevator elevator,
      Arm arm,
      Claw claw,
      Funnel funnel, /*Climber climber,*/
      CANdleSystem candle) {
    this.elevator = elevator;
    this.arm = arm;
    this.funnel = funnel;
    this.claw = claw;
    // this.climber = climber;
    this.candle = candle;

    state = Superstates.IDLE;
    level = Level.L1;

    ohtaniLaser = new LaserCan(34);

    // init is (channel/pin in roborio, max value in range, point which returns 0)
    // need to test to see where the 0 point is
    // elevatorEncoder = new DutyCycleEncoder(0, 1, 0);
    // armEncoder = new DutyCycleEncoder(1, 1, 0);
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Superstructure/State", state.toString());
    // Logger.recordOutput("Superstructure/Level", level.toString());

    // Logger.recordOutput("Superstructure/ElevatorEncoderConnection",
    // elevatorEncoder.isConnected());
    // Logger.recordOutput("Superstructure/ArmEncoderConnection", armEncoder.isConnected());
    // Logger.recordOutput("Superstructure/ElevatorEncoder", elevatorEncoder.get());
    // Logger.recordOutput("Superstructure/ArmEncoder", armEncoder.get());

    switch (state) {
      case IDLE:
        elevator.requestHeight(-0.055);
        arm.requestPosition(-.08);
        claw.requestIdle();
        candle.SetLEDGreen();

        // if (!funnel.requestClimb) {
        //   funnel.requestPosition(0.001);
        // }

        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        break;
      case PRE_FEED:
        elevator.requestHeight(-0.055);
        arm.requestPosition(-0.8);
        candle.SetLEDPurple();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (level == Level.L1 && requestScore) {
          state = Superstates.SCOREL1;
        } else if (level == Level.L2 && requestScore) {
          state = Superstates.SCOREL2;
        } else if (level == Level.L4 && requestScore) {
          state = Superstates.SCOREL4;
        } else if (requestScore) {
          state = Superstates.SCORE;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestFeed) {
          state = Superstates.FEEDING;
        }
        break;
      case FEEDING:
        funnel.requestFeed();
        claw.requestFeed();
        arm.requestPosition(-.08);
        elevator.requestHeight(-0.055);

        candle.SetLEDYellow();

        if (requestIdle) {
          state = Superstates.IDLE;
        } else if (level == Level.L1 && requestScore) {
          state = Superstates.SCOREL1;
        } else if (level == Level.L2 && requestScore) {
          state = Superstates.SCOREL2;
        } else if (level == Level.L4 && requestScore) {
          state = Superstates.SCOREL4;
        } else if (requestScore) {
          state = Superstates.SCORE;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        if (ohtaniLaser.getMeasurement().distance_mm <= 0.008) {
          state = Superstates.IDLE;
          unsetAllRequests();
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }
        break;
      case PRE_SCORE:
        if (level == Level.L1) {
          arm.requestPosition(.013);
          elevator.requestHeight(-.055);
          candle.SetLEDOrange();
        } else if (level == Level.L2) {
          arm.requestPosition(0.1193);
          elevator.requestHeight(-0.160);
          candle.SetLEDRed();
        } else if (level == Level.L3) {
          arm.requestPosition(0.715);
          elevator.requestHeight(-0.08);
          candle.SetLEDGreen();
        } else if (level == Level.L4) {
          arm.requestPosition(0.715);
          elevator.requestHeight(-0.84);
          candle.SetLEDYellow();
        }
        claw.requestIdle();

        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestIdle) {
          state = Superstates.IDLE;
        } else if (level == Level.L1 && requestScore) {
          state = Superstates.SCOREL1;
        } else if (level == Level.L2 && requestScore) {
          state = Superstates.SCOREL2;
        } else if (level == Level.L4 && requestScore) {
          state = Superstates.SCOREL4;
        } else if (requestScore) {
          state = Superstates.SCORE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        break;
      case SCOREL1:
        claw.requestShootL1();

        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        break;
      case SCOREL2:
        claw.requestShootL2();

        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestIdle) {
          state = Superstates.IDLE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        break;
      case SCORE:
        claw.requestShoot();
        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestIdle) {
          state = Superstates.IDLE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        break;
      case SCOREL4:
        claw.requestShootL4();
        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
        } else if (requestIdle) {
          state = Superstates.IDLE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestDisable) {
          state = Superstates.DISABLED;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        break;
        // case CLIMB_READY:
        //   candle.SetLEDPurple();

        //   climber.requestPosition(110);
        //   break;
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

  public void requestPreFeed() {
    unsetAllRequests();
    requestPreFeed = true;
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

  // public void requestClimbReady() {
  //   unsetAllRequests();
  //   requestClimbReady = true;
  // }

  public void requestDisable() {
    unsetAllRequests();
    requestDisable = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestPreFeed = false;
    requestFeed = false;
    requestPreScore = false;
    requestScore = false;
    // requestClimbReady = false;
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

  public void intakeCoral(Trigger action) {
    action.onTrue(
        new InstantCommand(
            () -> {
              requestFeed();
            }));
    action.onFalse(
        new InstantCommand(
            () -> {
              funnel.requestIdle();
              requestIdle();
            }));
  }

  public void readyIntakeCoral(Trigger action) {
    action.onTrue(
        new InstantCommand(
            () -> {
              requestPreFeed();
            }));
    action.onFalse(
        new InstantCommand(
            () -> {
              requestIdle();
            }));
  }

  public void intakeCoral(boolean runFeed, double time) {
    if (runFeed) {
      new InstantCommand(
              () -> {
                if (elevator.getHeight() < .5) {
                  requestFeed();
                }
              })
          .withTimeout(time);
    }
  }
}
