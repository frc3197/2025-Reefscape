// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managersubsystems;

import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.*;
import frc.robot.enums.*;

public class AlertManager extends SubsystemBase {

    private final LinkedList<AlertBody> alertList;


  public AlertManager() {
    alertList = new LinkedList<AlertBody>();
  }

  @Override
  public void periodic() {
    pollAlerts();
  }

  public void addAlert(AlertBody body) {
    if (!checkForExistingAlert(body.getAlertMode())) {
      alertList.add(body);
    }
  }

  public boolean checkForExistingAlert(AlertMode mode) {
    if(alertList.size() < 1) {
      return false;
    }

    for (AlertBody obj : alertList) {
      if (mode == obj.getAlertMode()) {
        return true;
      }
    }
    return false;
  }

  public boolean hasAlerts() {
    return alertList.size() > 0;
  }

  private void pollAlerts() {
    for (AlertBody obj : alertList) {
      if (obj.getAlertCompletion()) {
        alertList.remove(obj);
      }
    }
  }
}
