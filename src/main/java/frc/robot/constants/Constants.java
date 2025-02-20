// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final String kCANivore = "CANmeloAnthony";

  public static final boolean elevatorEnabled = true;
  public static final boolean armEnabled = true;
  public static final boolean funnelEnabled = true;
  public static final boolean climberEnabled = true;

  public static final boolean magneticLimitSwitchesEnabled = true;
  public static final boolean beamBreaksEnabled = false;

  public static final double NEO_FREE_SPEED = 6000.0; // TODO find the real free speed
  public static final int LED_NUM = 68; // TODO: Determine number of leds

  public class Elevator {

    // NEOS
    public static final int kElevatorPDH = 3; // Clockwise
    public static final int kElevatorRoboRio = 4; // Counterclockwise

    public static final double innerStageWeight = 12.0; //lbs

    public static final int EncoderDIO2 = 2;
    public static final int EncoderDIO3 = 3;

    public static final double gearRatio = 9.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.751); // pitch diameter

    public static final double setpointToleranceMeters = 0.01;

    public static final int supplyCurrentLimit = 40;

    public static final double peakForward = 0.5;
    public static final double peakReverse = -0.5;

    public static final double mechanismMaxAccel = 0.0;
    public static final double mechanismMaxCruiseVel = 0.0;

    public static final double homingVoltage = -1.0;
    public static final double homingVelocityThreshold = 0.01;
    public static final double homingThresholdSec = 0.25;

  }

  public class Manipulator {

    // NEO 550s
    public static final int kClaw = 6;

    // NEO
    public static final int kArm = 8;

    public class Arm {
          
    public static final double armFeedVoltage = 0.5; // TODO: Determine value

    }

    public class Claw {

      public static final double feedVoltage = 0.5; // TODO: Determine value
      public static final int currentLimit = 20;
      public static final double coralDetectionCurrentThreshold = 10.0; // Placeholder value, needs experimental determination

    }

  }

  public class Funnel {

    // NEO 550 
     public static final int kFunnelIntake = 5;

    // NEO
    public static final int kFunnelPivot = 7;

    public class Pivot {

      public static final int EncoderDIO0 = 0;
      public static final int EncoderDIO1 = 1;

      public static final double deployedSetpointMechanismRotations = 0.0;
      public static final double setpointToleranceMechanismRotations = 0.01;

      public static final double kP = 0.125; 
      public static final double kI = 0.0;
      public static final double kD = 0.025;
      public static final double kFF = 0.0; 
      public static final double minOutput = -0.4;
      public static final double maxOutput = 0.4; 

      public static final double motorGearRatio = 16.0;

      public static final int supplyCurrentLimit = 40;
      
      // Wrap to 0 at threshold assuming pivot is pushed back hard against zero point hardstop
      public static final double absZeroWrapThreshold = 0.95;

    }

    
    public class Intake {

      public static final double motorGearRatio = 4.0;

      public static final int supplyCurrentLimit = 20;

    }
    
  }

    public class Climber {
  
    // NEO
    public static final int kNearL1 = 9;
    public static final int kNearFunnel = 10;

    public static final double gearRatio = 80.0;
    public static final double splineXLDiameter = Units.inchesToMeters(1.37795);
    public static final double setpointToleranceMeters = 0.01;
    public static final int supplyCurrentLimit = 40;

    
    public static final double stowedPoistion = 0;
    public static final double climbingPosition = 20; // TODO find real position

  }

  public class L1Mechanism {

    // NEO
    public static final int kL1Deploy = 25;

    // NEO 550
    public static final int kL1Intake = 26;

  }

  public class CANdle {
    public static final int kCANdleID = 31;
    
    // public static final int MaxBrightnessAngle = 90;
    // public static final int MidBrightnessAngle = 180;
    // public static final int ZeroBrightnessAngle = 270;


  }
  public static class Scoring {
    public static final double L2ScoringHeight = Units.inchesToMeters(4);
    public static final double L4ScoringHeight = Units.inchesToMeters(28);

    public static final double L2SafeFlipHeight =
        0.240; // elv position to go to initially for pivot to flip
    public static final double safeFlipPosition =
        0.217; // position to retract elevator at when flipper in reef
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}