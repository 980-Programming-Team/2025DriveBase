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

  // requests
  private boolean requestIdle;
  private boolean requestPreFeed;
  private boolean requestFeed;
  private boolean requestAutoFeed;
  private boolean requestPreScore;
  private boolean requestScore;
  // private boolean requestClimbReady;
  private boolean requestDisable;

  // subsystems
  private Superstates state;
  private Elevator elevator;
  public static Arm arm;
  public Funnel funnel;
  private Claw claw;
  // private Climber climber;

  // LEDs
  public CANdleSystem candle;

  // Elevator Height
  private Level level;

  // Coral Detection
  public static LaserCan ohtaniLaser;

  // Coral auto feed into position timer
  private Timer feedSetTimer;

  // pretty sure that both encoders turn counterclockwise
  // if negative turning for positive movement, use setInverted()
  private DutyCycleEncoder elevatorEncoder;
  private DutyCycleEncoder armEncoder;

  // States of the Superstructure
  // -> tells robot what to do at current moment so actions don't conflict
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
    //// Debugging commented out but can be reentered whenever
    //// -> Issue is it uses a lot of memory, use only if needed:

    // Logger.recordOutput("Superstructure/State", state.toString());
    // Logger.recordOutput("Superstructure/Level", level.toString());

    // Logger.recordOutput("Superstructure/ElevatorEncoderConnection",
    // elevatorEncoder.isConnected());
    // Logger.recordOutput("Superstructure/ArmEncoderConnection", armEncoder.isConnected());
    // Logger.recordOutput("Superstructure/ElevatorEncoder", elevatorEncoder.get());
    // Logger.recordOutput("Superstructure/ArmEncoder", armEncoder.get());

    switch (state) {
      case IDLE:
        //   ^^^^ if robot is idle then
        //   vvvv <- right click any request____ of a subsystem and click "Go To Definiton"
        //           then scroll up to their "periodic" function to see how each is activated
        // through their own state machine
        //           works just like this one with requests to call actions to be done
        elevator.requestHeight(-0.048);
        arm.requestPosition(2.05); // -0.8
        claw.requestIdle();
        candle.SetLEDOrange();

        // vvvvvvvvvvvv <- each case needs to check for if any other request is initiated to leave
        // its current action and avoid conflict
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
        //   ^^^^^^^^ if operator pressess button for arm to be ready to be fed, then vvvv (and so
        // on for each case)
        elevator.requestHeight(-0.048);
        arm.requestPosition(2.05);
        candle.SetLEDL2();

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
        arm.requestPosition(2.09); // -.08 was 2.05z
        elevator.requestHeight(-0.048);

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

        // detects that we have coral through laserCan
        if (hasCoral()) {
          if (feedSetTimer.get() <= 0) {
            feedSetTimer.start();
          }

          // sets coral to good position in claw
          if (feedSetTimer.get() >= 0.085) {
            state = Superstates.IDLE;
            feedSetTimer.stop();
            feedSetTimer.reset();
            unsetAllRequests();
          }
        }
        break;
      case AUTO_FEEDING: // auto feed time is slower due to max accel on other motos, had to
        // increase
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

        if (hasCoral()) {
          if (feedSetTimer.get() <= 0) {
            feedSetTimer.start();
          }

          if (feedSetTimer.get() >= 0.105) {
            state = Superstates.IDLE;
            feedSetTimer.stop();
            feedSetTimer.reset();
            unsetAllRequests();
          }
        }
        break;
      case PRE_SCORE:
        if (level == Level.L1) {
          arm.requestPosition(2.20); // 0.013
          elevator.requestHeight(-.055);
          candle.SetLEDL1();
        } else if (level == Level.L2) {
          arm.requestPosition(2.36); // 0.1193
          elevator.requestHeight(-0.252);
          candle.SetLEDL2();
        } else if (level == Level.L3) {
          arm.requestPosition(4.03);
          elevator.requestHeight(-0.08);
          candle.SetLEDL3();
        } else if (level == Level.L4) {
          arm.requestPosition(4.10); // 0.715
          elevator.requestHeight(-0.84);
          candle.SetLEDL4();
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

  public static boolean hasCoral() {
    if (ohtaniLaser.getMeasurement().distance_mm <= 0.008) {
      return true;
    }

    return false;
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

  // gets default command using driver controller
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
}
