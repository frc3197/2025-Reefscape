// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.enums.AlignRequestType;
import frc.robot.enums.RobotMode;
import frc.robot.subsystems.Align;

public class AlignReef extends Command {

  private Align align;
  private AlignRequestType requestType;
  private ChassisSpeeds maximumSpeeds;
  private Translation3d goalErrors;

  private int section = -1;

  private boolean aligned = false;

  public AlignReef(Align align, AlignRequestType requestType, ChassisSpeeds maximumSpeeds, Translation3d goalErrors) {
    this.align = align;
    this.requestType = requestType;
    this.maximumSpeeds = maximumSpeeds;
    this.goalErrors = goalErrors;
  }

  public AlignReef(Align align, AlignRequestType requestType, ChassisSpeeds maximumSpeeds, Translation3d goalErrors,
      int section) {
    this.align = align;
    this.requestType = requestType;
    this.maximumSpeeds = maximumSpeeds;
    this.goalErrors = goalErrors;
    this.section = section;
  }

  @Override
  public void initialize() {
    //section = -1;
    RobotContainer.setRobotMode(RobotMode.ALIGN_REEF_CUSTOM);
  }

  @Override
  public void execute() {
    aligned = align.runCustomAlign(this.requestType, maximumSpeeds, goalErrors, section);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.setRobotMode(RobotMode.NONE);
  }

  @Override
  public boolean isFinished() {
    return aligned;
  }
}
