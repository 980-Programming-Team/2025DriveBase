package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SourceManager {
  private CommandXboxController driver;

  public SourceManager(int port) {
    driver = new CommandXboxController(port);
  }

  private double[] autoRotateBlue = {
    0, Math.PI / 3, 2 * Math.PI / 3, Math.PI, -2 * Math.PI / 3, -Math.PI / 3,
  };

  private double[] autoRotateRed = {
    Math.PI, -2 * Math.PI / 3, -Math.PI / 3, 0, Math.PI / 3, 2 * Math.PI / 3,
  };

  private int[] aprilTagRed = {
    1, 2,
  };

  private int[] aprilTagBlue = {
    13, 12,
  };

  private double autoRotatePosition;
  private int aprilTag;

  private boolean flipReqested;

  // scoring face enumerated from 0 - 5 counterclockwise starting at reef face
  // closest to middle driver station for blue and red
  public void setSourcePosition(int source) {
    if (Robot.alliance == DriverStation.Alliance.Blue) {
      this.autoRotatePosition = autoRotateBlue[source];
      this.aprilTag = aprilTagBlue[source];
    } else {
      this.autoRotatePosition = autoRotateRed[source];
      this.aprilTag = aprilTagRed[source];
    }
  }

  public void setFlipRequest(boolean requestFlip) {
    flipReqested = requestFlip;
  }

  public CommandXboxController getDriver() {
    return driver;
  }

  // public GenericHID getRightController() {
  //   return rightController;
  // }

  // public GenericHID getLeftController() {
  //   return leftController;
  // }

  public int getAprilTag() {
    return aprilTag;
  }

  public double getAutoRotatePosition() {
    return autoRotatePosition;
  }

  public boolean getFlipRequested() {
    return flipReqested;
  }

  public void configScoringPosButtons() {
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(0);
                }));
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(0);
                }));

    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(1);
                }));
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(1);
                }));

    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(2);
                }));
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(2);
                }));

    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(3);
                }));
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(3);
                }));

    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(4);
                }));
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(4);
                }));

    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(5);
                }));
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  setSourcePosition(5);
                }));
  }
}
