package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Arm.HomeArm;
import frc.robot.commands.Arm.SupplyArmSpeed;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Pivot.HomePivot;
import frc.robot.commands.Pivot.SupplyPivotSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Dashboard;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final Pivot pivot = new Pivot();
  private final Arm arm = new Arm();
  private final Manipulator manipulator = new Manipulator();

  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  public RobotContainer() {
    configureBindings();
    pivot.setDefaultCommand(new SupplyPivotSpeed(() -> operator.getRawAxis(1), pivot));
    arm.setDefaultCommand(new SupplyArmSpeed(() -> -operator.getRawAxis(5), arm));
    drivetrain.setDefaultCommand(new ArcadeDrive(() -> -driver.getRawAxis(1), () -> -driver.getRawAxis(2), drivetrain));
  }

  private void configureBindings() {
    JoystickButton togglePincher = new JoystickButton(operator, 1);
    JoystickButton toggleWrist = new JoystickButton(operator, 3);

    togglePincher.onTrue(new InstantCommand(() -> manipulator.togglePincher()));
    toggleWrist.onTrue(new InstantCommand(() -> manipulator.toggleWrist()));

    Dashboard.putSendable("Home Pivot", new HomePivot(pivot));
    Dashboard.putSendable("Home Arm", new HomeArm(arm));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
