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

  public static final boolean elevatorEnabled = true;
  public static final boolean armEnabled = true;
  public static final boolean funnelEnabled = true;

  public static final boolean magneticLimitSwitchesEnabled = true;
  public static final boolean beamBreaksEnabled = false;

  public static final double KRAKEN_FREE_SPEED = 6000.0;
  public static final int LED_NUM = 68; // TODO: Determine number of leds

  public class Elevator {

    // Krakens
    public static final int kElevatorPDP = 3; // Clockwise
    public static final int kElevatorRoboRio = 4; // Counterclockwise

    public static final double gearRatio = 9.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.751); // pitch diameter

    public static final double setpointToleranceMeters = 0.01;

    public static final double supplyCurrentLimit = 40;
    public static final double statorCurrentLimit = 100;

    public static final double peakForwardVoltage = 11.5;
    public static final double peakReverseVoltage = -11.5;

    public static final InvertedValue roboRioMotorInversion = InvertedValue.Clockwise_Positive;

    // L1 height gains
    public static final double kS0 = 0;
    public static final double kP0 = 2.0;
    public static final double kD0 = 0.05;

    // L2 height gains
    public static final double kS1 = 0.4;
    public static final double kP1 = 2.0;
    public static final double kD1 = 0;
    
    // L3 height gains
    public static final double kS2 = 0;
    public static final double kP2 = 2.0;
    public static final double kD2 = 0.05;
    public static final double kG2 = 0.40;

  }

  public class Arm {

    // NEO 550s
    public static final int kClaw = 6;

    // NEO
    public static final int kArm = 8;

    public static final double coralDetectionCurrentThreshold = 10.0; // Placeholder value, needs experimental determination
    
    public static final double armFeedVoltage = 0.5; // TODO: Determine value
    public static final double clawFeedVoltage = 0.5; // TODO: Determine value

  }

  public class Funnel {

    // NEO 550 
    public static final int kFunnelIntake = 5;

    // NEO
    public static final int kFunnelPivot = 7;
    
  }

  public class Cage {

    // NEO
    public static final int kCageL1 = 9;
    public static final int kCageFunnel = 10;
  }

  public class L1Mechanism {

    // NEO
    public static final int kL1Deploy = 25;

    // NEO 550
    public static final int kL1Intake = 26;

  }

  public class CANdle {
    public static final int kCANdleID = 31;
    
    public static final int MaxBrightnessAngle = 90;
    public static final int MidBrightnessAngle = 180;
    public static final int ZeroBrightnessAngle = 270;

  }


  public static final int MaxBrightnessAngle = 90;
  public static final int MidBrightnessAngle = 180;
  public static final int ZeroBrightnessAngle = 270;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
