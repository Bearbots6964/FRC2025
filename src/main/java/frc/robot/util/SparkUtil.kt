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

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import java.util.function.Consumer
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import java.util.function.Supplier

object SparkUtil {
    /** Stores whether any error was has been detected by other utility methods.  */
    var sparkStickyFault: Boolean = false

    /** Processes a value from a Spark only if the value is valid.  */
    fun ifOk(spark: SparkBase, supplier: DoubleSupplier, consumer: DoubleConsumer) {
        val value = supplier.asDouble
        if (spark.lastError == REVLibError.kOk) {
            consumer.accept(value)
        } else {
            sparkStickyFault = true
        }
    }

    /** Processes a value from a Spark only if the value is valid.  */
    fun ifOk(
        spark: SparkBase, suppliers: Array<DoubleSupplier>, consumer: Consumer<DoubleArray?>
    ) {
        val values = DoubleArray(suppliers.size)
        for (i in suppliers.indices) {
            values[i] = suppliers[i].asDouble
            if (spark.lastError != REVLibError.kOk) {
                sparkStickyFault = true
                return
            }
        }
        consumer.accept(values)
    }

    /** Attempts to run the command until no error is produced.  */
    @JvmStatic
    fun tryUntilOk(spark: SparkBase?, maxAttempts: Int, command: Supplier<REVLibError>) {
        for (i in 0..<maxAttempts) {
            val error = command.get()
            if (error == REVLibError.kOk) {
                break
            } else {
                sparkStickyFault = true
            }
        }
    }
}