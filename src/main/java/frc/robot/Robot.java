package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Arm.supplyArmSpeed;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Pivot.supplyPivotSpeed;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (m_robotContainer.drivetrain.getCurrentCommand() != null) {
      SmartDashboard.putString("Current Command", m_robotContainer.drivetrain.getCurrentCommand().getName());
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.manipulator.updateRedPattern();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    CommandScheduler.getInstance().removeDefaultCommand(m_robotContainer.drivetrain);
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.drivetrain.differentialDrive.feed();
    m_robotContainer.manipulator.updateRedPattern();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setDefaultCommands();

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
