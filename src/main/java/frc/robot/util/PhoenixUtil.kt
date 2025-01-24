// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.util

import com.ctre.phoenix6.StatusCode
import java.util.function.Supplier

object PhoenixUtil {
    /** Attempts to run the command until no error is produced.  */
    @JvmStatic
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
        for (i in 0..<maxAttempts) {
            val error = command.get()
            if (error.isOK) break
        }
    }
}
