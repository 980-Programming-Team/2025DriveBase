// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {
  /** Creates a new Intake. */
  CollectorSub Collector;

  public Intake(CollectorSub collect) {
    // Use addRequirements() here to declare subsystem dependencies.
    Collector = collect;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Collector.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Collector.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Collector.off();
    return false;
  }
}
