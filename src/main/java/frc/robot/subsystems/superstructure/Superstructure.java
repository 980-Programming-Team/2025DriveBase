// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public enum SuperstructureState {
    START,
    STOWED
  }

  private Elevator Elevator;
  private Funnel Funnel;
  private Claw Claw;
  private Arm Arm;

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private EdgeCommand edgeCommand;

  private SuperstructureState state = SuperstructureState.START;
  private SuperstructureState next = null;
  private SuperstructureState goal = SuperstructureState.START;

  @AutoLogOutput(key = "Superstructure/EStopped")
  private boolean isEStopped = false;

  private BooleanSupplier disabledOverride = () -> false;
  private final Alert driverDisableAlert =
      new Alert("Superstructure disabled due to driver override.", Alert.AlertType.kWarning);
  private final Alert emergencyDisableAlert =
      new Alert(
          "Superstructure emergency disabled due to high position error. Disable the superstructure manually and reenable to reset.",
          Alert.AlertType.kError);

  public Superstructure(Elevator elevator, Funnel funnel, Claw claw, Arm arm) {
    Elevator = elevator;
    Funnel = funnel;
    Claw = claw;
    Arm = arm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Run periodic
    Elevator.periodic();
    Funnel.periodic();
    Claw.periodic();
    Arm.periodic();

    if (DriverStation.isDisabled()) {
      next = null;
    } else if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
      // Update edge to new state
      if (next != null) {
        state = next;
        next = null;
      }

      // Schedule next command in sequence
      if (state != goal) {
        bfs(state, goal)
            .ifPresent(
                next -> {
                  this.next = next;
                  edgeCommand = graph.getEdge(state, next);
                  edgeCommand.getCommand().schedule();
                });
      }
    }

    // Tell Elevator we are stowed
    Elevator.stowed = state == SuperstructureState.STOWED;

    // E Stop Elevator if Necessary
    isEStopped = isEStopped && Constants.getMode() != Mode.SIM;
    Elevator.isEStopped = isEStopped;

    driverDisableAlert.set(disabledOverride.getAsBoolean());
    emergencyDisableAlert.set(isEStopped);

    // Log state
    Logger.recordOutput("Superstructure/State", state);
    Logger.recordOutput("Superstructure/Next", next);
    Logger.recordOutput("Superstructure/Goal", goal);
    if (edgeCommand != null) {
      Logger.recordOutput(
          "Superstructure/EdgeCommand",
          graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
    } else {
      Logger.recordOutput("Superstructure/EdgeCommand", "");
    }
  }
  
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    private final boolean restricted = false;
    private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  private enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
    // Map to track the parent of each visited node
    HashMap<SuperstructureState, SuperstructureState> parents = new HashMap<>();
    Queue<SuperstructureState> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      SuperstructureState current = queue.poll();
      // Check if we've reached the goal
      if (current == goal) {
        break;
      }
      // Process valid neighbors
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        SuperstructureState neighbor = graph.getEdgeTarget(edge);
        // Only process unvisited neighbors
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }


    private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
      return (!edge.isRestricted() || goal == graph.getEdgeTarget(edge))
          && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
              || dispenser.hasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE));
    }
}
