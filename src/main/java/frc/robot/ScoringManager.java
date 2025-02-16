package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class ScoringManager {
  private GenericHID controller;

  // private GenericHID rightController;
  // private GenericHID leftController;

  public ScoringManager(int port) {
    controller = new GenericHID(port);
    
    // rightController = new GenericHID(rightPort);
    // leftController = new GenericHID(leftPort);
  }

  private double[] autoRotateBlue = {
    0, Math.PI / 3, 2 * Math.PI / 3, Math.PI, -2 * Math.PI / 3, -Math.PI / 3,
  };

  private double[] autoRotateRed = {
    Math.PI, -2 * Math.PI / 3, -Math.PI / 3, 0, Math.PI / 3, 2 * Math.PI / 3,
  };

  private int[] aprilTagRed = {
    7, 8, 9, 10, 11, 6,
  };

  private int[] aprilTagBlue = {
    18, 17, 22, 21, 20, 19,
  };

  private double autoRotatePosition;
  private int aprilTag;

  private boolean flipReqested;

  private int scoringLocation = 1;
  private String alignDirection = "AL";
  private String scoringLevel = "L2";

  // scoring face enumerated from 0 - 5 counterclockwise starting at reef face
  // closest to middle driver station for blue and red
  public void setScoringPosition(int scoringFace) {
    if (Robot.alliance == DriverStation.Alliance.Blue) {
      this.autoRotatePosition = autoRotateBlue[scoringFace];
      this.aprilTag = aprilTagBlue[scoringFace];
    } else {
      this.autoRotatePosition = autoRotateRed[scoringFace];
      this.aprilTag = aprilTagRed[scoringFace];
    }
  }

  public void setFlipRequest(boolean requestFlip) {
    flipReqested = requestFlip;
  }

  public GenericHID getController() {
    return controller;
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

  public void updateScoringList() {
    String[] scoringList = new String[3];
    scoringList[0] = getScoringLocationName(scoringLocation);
    scoringList[1] = alignDirection;
    scoringList[2] = scoringLevel;

    Logger.recordOutput("Scoring/ScoringList", scoringList);
  }

  public void clearScoringList() {
    Logger.recordOutput("Scoring/ScoringList", new String[0]);
  }

  private String getScoringLocationName(int location) {
    switch (location) {
      case 1:
        return "Front Center";
      case 2:
        return "Front Right";
      case 3:
        return "Back Right";
      case 4:
        return "Back Center";
      case 5:
        return "Back Left";
      case 6:
        return "Front Left";
      default:
        return "Unknown";
    }
  }

  public void setScoringLocation(int location) {
    this.scoringLocation = location;
    updateScoringList();
  }

  public void setAlignDirection(String direction) {
    this.alignDirection = direction;
    updateScoringList();
  }

  public void setScoringLevel(String level) {
    this.scoringLevel = level;
    updateScoringList();
  }

  public void configScoringPosButtons() {
    new JoystickButton(controller, 1)
        .onTrue(new InstantCommand(() -> setScoringLocation(1)));
    new JoystickButton(controller, 2)
        .onTrue(new InstantCommand(() -> setScoringLocation(2)));
    new JoystickButton(controller, 3)
        .onTrue(new InstantCommand(() -> setScoringLocation(3)));
    new JoystickButton(controller, 4)
        .onTrue(new InstantCommand(() -> setScoringLocation(4)));
    new JoystickButton(controller, 5)
        .onTrue(new InstantCommand(() -> setScoringLocation(5)));
    new JoystickButton(controller, 6)
        .onTrue(new InstantCommand(() -> setScoringLocation(6)));

    new JoystickButton(controller, 7)
        .onTrue(new InstantCommand(() -> setAlignDirection("AL")));
    new JoystickButton(controller, 8)
        .onTrue(new InstantCommand(() -> setAlignDirection("AR")));

    new JoystickButton(controller, 9)
        .onTrue(new InstantCommand(() -> setScoringLevel("L2")));
    new JoystickButton(controller, 10)
        .onTrue(new InstantCommand(() -> setScoringLevel("L3")));
    new JoystickButton(controller, 11)
        .onTrue(new InstantCommand(() -> setScoringLevel("L4")));
  }
}
