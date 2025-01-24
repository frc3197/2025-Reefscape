// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.BooleanSupplier;

import frc.robot.enums.ErrorMode;

public class ErrorBody {

    private final ErrorMode errorMode;
    private final BooleanSupplier errorComplete;

    public ErrorBody(ErrorMode mode, BooleanSupplier completion) {
        this.errorMode = mode;
        this.errorComplete = completion;
    }

    public ErrorMode getErrorMode() {
        return this.errorMode;
    }

    public boolean getErrorCompletion() {
        return this.errorComplete.getAsBoolean();
    }

}
