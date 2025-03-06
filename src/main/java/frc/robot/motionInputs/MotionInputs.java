package frc.robot.motionInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

import static edu.wpi.first.wpilibj2.command.Commands.run;

public class MotionInputs {
  public Queue<MotionInput> buffer;
  public ArrayList<Integer> held;

  public static int maxBufferLength = 61;

  public MotionInputs() {
    buffer = new LinkedList<>();
    held = new ArrayList<>();
  }

  public Command update(CommandXboxController controller) {
    return run(
        () -> {
          buffer.add(
                  new MotionInput(-controller.getLeftX(), -controller.getRightX()));
          if(buffer.peek().getLife() <= 0) buffer.poll();
          for (MotionInput input : buffer) {
            input.update();
          }
        });
  }

  public Command runCommand(Arm arm, Elevator elevator) {
    int temp;
    for (int i = 0; i < buffer.size(); i++) {
      temp = buffer.peek().getNumpadNotation();
      if (temp == 0 || !held.isEmpty() && temp == held.get(held.size() - 1)) continue;
      held.add(temp);
    }

    if (held.size() == 3) {
      // 214P input
      if (held.get(0) == 2 && held.get(1) == 1 && held.get(2) == 4) {
        held.clear();
        return run(
            () ->
                elevator
                    .goToPosition(60.0)
                    .andThen(arm.moveArmToAngle(10.0))
                    .andThen(elevator.goToPosition(40.0))
                    .andThen(elevator.goToPosition(53.7))
                    .andThen(arm.moveArmToAngle(25.0))
                    .andThen(elevator.goToPosition(10.0).alongWith(arm.moveArmToAngle(170.0))));
      }
    }

    return run(() -> held.clear());
  }
}
