package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Arm.AlternateHomeArm;
import frc.robot.commands.Arm.HomeArm;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.supplyArmSpeed;
import frc.robot.commands.Pivot.HomePivot;
import frc.robot.commands.Pivot.SetPivotPosition;
import frc.robot.commands.Pivot.supplyPivotSpeed;
import frc.robot.commands.auton.HighCube;
import frc.robot.commands.auton.HighCubeBalance;
import frc.robot.commands.auton.balance.ForwardBalance;
import frc.robot.commands.auton.balance.ReverseBalance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.utils.Dashboard;

public class RobotContainer {

  public final Drivetrain drivetrain = new Drivetrain();
  private final Pivot pivot = new Pivot(this);
  private final Arm arm = new Arm();
  private final Manipulator manipulator = new Manipulator();

  public final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    configureBindings();
    pivot.setDefaultCommand(new supplyPivotSpeed(() -> -operator.getRawAxis(1), pivot));
    arm.setDefaultCommand(new supplyArmSpeed(() -> -operator.getRawAxis(5), arm));

    autoChooser.setDefaultOption("Forward balance", new ForwardBalance(drivetrain));
    autoChooser.addOption("Empty", new InstantCommand());
    autoChooser.addOption("Reverse Balance", new ReverseBalance(drivetrain));
    autoChooser.addOption("High Cube Balance", new HighCubeBalance(drivetrain, pivot, arm, manipulator));
    autoChooser.addOption("High Cube", new HighCube(drivetrain, pivot, arm, manipulator));
    Dashboard.putSendable("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Driver button bindings

    // Operator button bindigns
    JoystickButton openPincher = new JoystickButton(operator, 7);
    JoystickButton closePincher = new JoystickButton(operator, 5);
    JoystickButton wristUp = new JoystickButton(operator, 6);
    JoystickButton wristDown = new JoystickButton(operator, 8);
    JoystickButton requestCone = new JoystickButton(operator, 3);
    JoystickButton requestCube = new JoystickButton(operator, 2);
    JoystickButton retractArm = new JoystickButton(operator, 15);
    JoystickButton homePivot = new JoystickButton(operator, 9);
    JoystickButton homeArm = new JoystickButton(operator, 10);
    


    openPincher.onTrue(new InstantCommand(() -> manipulator.openPincher()));
    closePincher.onTrue(new InstantCommand(() -> manipulator.closePincher()));
    wristUp.onTrue(new InstantCommand(() -> manipulator.wristUp()));
    wristDown.onTrue(new InstantCommand(() -> manipulator.wristDown()));
    requestCone.onTrue(new InstantCommand(() -> manipulator.requestCone()));
    requestCube.onTrue(new InstantCommand(() -> manipulator.requestCube()));
    retractArm.onTrue(new AlternateHomeArm(pivot, arm));
    homePivot.onTrue(new HomePivot(pivot, arm, manipulator));
    homeArm.onTrue(new HomeArm(pivot, arm));

    new JoystickButton(operator, 1).whileTrue(new SetPivotPosition(15, pivot));
    new POVButton(operator, 180).whileTrue(new SetArmPosition(0.2, arm));
    
    
    
    Dashboard.putSendable("Home Pivot", new HomePivot(pivot, arm, manipulator));
    Dashboard.putSendable("Home Arm", new HomeArm(pivot, arm));
    Dashboard.putSendable("Reset Pivot", new InstantCommand(() -> pivot.resetPivotEncoder(84)).withName("Reset Pivot"));
    Dashboard.putSendable("ALTERNATE ARM", new AlternateHomeArm(pivot, arm));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
