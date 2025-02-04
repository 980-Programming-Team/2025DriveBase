// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Reef;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LevelFour extends Command {
  /** Creates a new LeftLevelFour. */
  private Elevator Elevator;

  public LevelFour(Elevator elevator) {
    Elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator.Auto(4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Elevator.GetEncPosition() >= 80) return true;
    return false;
  }
}
