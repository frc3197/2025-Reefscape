// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managersubsystems;

import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ErrorBody;
import frc.robot.enums.ErrorMode;

public class ErrorManager extends SubsystemBase {

  private final LinkedList<ErrorBody> errorList;

  public ErrorManager() {
    errorList = new LinkedList<ErrorBody>();
  }

  @Override
  public void periodic() {
    pollErrors();
  }

  public void addError(ErrorBody body) {
    if (!checkForExistingError(body.getErrorMode())) {
      errorList.add(body);
    }
  }

  public boolean checkForExistingError(ErrorMode mode) {
    for (ErrorBody obj : errorList) {
      if (mode == obj.getErrorMode()) {
        return true;
      }
    }
    return false;
  }

  private void pollErrors() {
    for (ErrorBody obj : errorList) {
      if (obj.getErrorCompletion()) {
        errorList.remove(obj);
      }
    }
  }
}
