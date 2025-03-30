package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
  private boolean requestAutoFeed;
  private boolean requestPreScore;
  private boolean requestScore;
  // private boolean requestClimbReady;
  private boolean requestDisable;

  private Superstates state;
  private Elevator elevator;
  public static Arm arm;
  public Funnel funnel;
  private Claw claw;
  // private Climber climber;

  public CANdleSystem candle;

  private Level level;

  public static LaserCan ohtaniLaser;

  private Timer feedSetTimer;

  // pretty sure that both encoders turn counterclockwise
  // if negative turning for positive movement, use setInverted()
  private DutyCycleEncoder elevatorEncoder;
  private DutyCycleEncoder armEncoder;

  public static enum Superstates {
    IDLE,
    PRE_FEED,
    FEEDING,
    AUTO_FEEDING,
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

    feedSetTimer = new Timer();
    feedSetTimer.stop();
    feedSetTimer.reset();
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
        arm.requestPosition(2.05); // -0.8
        claw.requestIdle();
        candle.SetLEDOrange();

        // if (!funnel.requestClimb) {
        //   funnel.requestPosition(0.001);
        // }

        if (requestFeed) {
          state = Superstates.FEEDING;
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
        } else if (requestPreScore) {
          state = Superstates.PRE_SCORE;
          // } else if (requestClimbReady) {
          // state = Superstates.CLIMB_READY;
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }

        break;
      case PRE_FEED:
        elevator.requestHeight(-0.045);
        arm.requestPosition(2.05); // -0.8
        candle.SetLEDPurple4();

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
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
        }
        break;
      case FEEDING:
        funnel.requestFeed();
        claw.requestFeed();
        arm.requestPosition(2.05); // -.08
        elevator.requestHeight(-0.045);

        candle.SetLEDPurple0();

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
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
        }

        if (ohtaniLaser.getMeasurement().distance_mm <= 0.008) {
          if (feedSetTimer.get() <= 0) {
            feedSetTimer.start();
          }

          if (feedSetTimer.get() >= 0.1) {
            state = Superstates.IDLE;
            feedSetTimer.stop();
            feedSetTimer.reset();
            unsetAllRequests();
          }
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }
        break;
      case AUTO_FEEDING:
        funnel.requestAutoFeed();
        claw.requestAutoFeed();
        arm.requestPosition(2.05); // -.08
        elevator.requestHeight(-0.045);

        candle.SetLEDPurple0();

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
          if (feedSetTimer.get() <= 0) {
            feedSetTimer.start();
          }

          if (feedSetTimer.get() >= 0.125) {
            state = Superstates.IDLE;
            feedSetTimer.stop();
            feedSetTimer.reset();
            unsetAllRequests();
          }
        } else if (requestPreFeed) {
          state = Superstates.PRE_FEED;
        }
        break;
      case PRE_SCORE:
        if (level == Level.L1) {
          arm.requestPosition(2.20); // 0.013
          elevator.requestHeight(-.055);
          candle.SetLEDPurple1();
        } else if (level == Level.L2) {
          arm.requestPosition(2.36); // 0.1193
          elevator.requestHeight(-0.245);
          candle.SetLEDPurple2();
        } else if (level == Level.L3) {
          arm.requestPosition(4.03);
          elevator.requestHeight(-0.08);
          candle.SetLEDPurple3();
        } else if (level == Level.L4) {
          arm.requestPosition(4.10); // 0.715
          elevator.requestHeight(-0.84);
          candle.SetLEDPurple4();
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
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
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
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
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
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
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
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
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
        } else if (requestAutoFeed) {
          state = Superstates.AUTO_FEEDING;
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

  public void requestAutoFeed() {
    unsetAllRequests();
    requestAutoFeed = true;
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
    requestAutoFeed = false;
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

  // public void intakeCoral(/*boolean runFeed, double time*/) {
  //   // if (runFeed) {
  //   //   new InstantCommand(
  //   //           () -> {
  //   //             if (elevator.getHeight() < .5) {
  //                 requestFeed();
  //         //       }
  //         //     })
  //         // .withTimeout(time);
  //   // }
  // }
}
