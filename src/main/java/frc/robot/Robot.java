// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomous_command;

  private final RobotContainer robot_container;
  private final Vision vision;

  public Robot() {
    robot_container = new RobotContainer();
    vision = new Vision(robot_container.drivetrain);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // vision.updateVision();
    // field.setRobotPose(robot_container.drivetrain.getPose());
  }
  
  @Override
  public void disabledInit() {
  }
  
  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }
  
  @Override
  public void autonomousInit() {
    autonomous_command = robot_container.getAutonomousCommand();
    
    if (autonomous_command != null) {
      autonomous_command.schedule();
    }

    robot_container.allianceFlipPoses();

    // FieldConstants.Reef.initPoses();
  }
  
  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void autonomousExit() {
  }
  
  @Override
  public void teleopInit() {
    // robot_container.configureBindings();
    CommandScheduler.getInstance().cancelAll();

    if (autonomous_command != null) {
      autonomous_command.cancel();
    }

    robot_container.allianceFlipPoses();

    // FieldConstants.Reef.inwitPoses();
  }
  
  @Override
  public void teleopPeriodic() {
  }
  
  @Override
  public void teleopExit() {
  }
  
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
