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

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public class SuperstructureConstants {

    // Krakens
    public static final double kElevatorPDP = 3;
    public static final double kElevatorRoboRio = 4;

    // NEO 550s
    public static final double kFunnelIntake = 5;
    public static final double kClaw = 6;

    // NEO
    public static final double kFunnel = 7;
    public static final double kArm = 8;
  }

  public class CageConstants {

    // NEO
    public static final double kCageL1 = 9;
    public static final double kCageFunnel = 10;
  }

  public class L1Mechanism {

    // NEO
    public static final double kL1Deploy = 25;

    // NEO 550
    public static final double kL1Collect = 26;
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
