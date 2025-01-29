// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.enums.*;
import frc.robot.util.*;

/** Add your docs here. */
public class AlertBody {

    private final AlertMode alertMode;
    private BooleanSupplier alertComplete;
    private final double startTime;
    private final double totalTime;

    public AlertBody(AlertMode mode, double totalTime) {
        this.alertMode = mode;
        this.totalTime = totalTime;
        this.startTime = Timer.getTimestamp();

        this.alertComplete = () -> {
            return Timer.getTimestamp() > this.startTime + this.totalTime;
        };

    }

    public AlertMode getAlertMode() {
        return this.alertMode;
    }

    public boolean getAlertCompletion() {
        return this.alertComplete.getAsBoolean();
    }

}
