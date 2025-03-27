package frc.robot.util

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Robot

class Polygon(private val vertices: List<Translation2d>) {
    
    private var boundingBox: Rectangle2d
    init {
        val alliance = DriverStation.getAlliance().get()
        if (alliance == DriverStation.Alliance.Red) {
            vertices.forEach {
                FlippingUtil.flipFieldPosition(it)
            }
        }
        var left = Units.Feet.of(1e6) // arbitrary large number
        var right = Units.Feet.of(-1e6)
        var top = Units.Feet.of(1e6)
        var bottom = Units.Feet.of(-1e6)
        
        for (vertex in vertices) {
            left = Measure.min(left, vertex.measureX) as Distance
            right = Measure.max(right, vertex.measureX) as Distance
            top = Measure.min(top, vertex.measureY) as Distance
            bottom = Measure.max(bottom, vertex.measureY) as Distance
        }
        
        val topLeft = Translation2d(left, top)
        val bottomRight = Translation2d(right, bottom)
        boundingBox = Rectangle2d(topLeft, bottomRight)
    }



    /**
     * This was copied from the Polygon class in the java.awt package.
     * And then converted to Kotlin.
     * Needless to say, here be dragons.
     */
    fun contains(point: Translation2d): Boolean {
        if (vertices.size <= 2 || !boundingBox.contains(point)) {
            return false
        }
        var hits = 0

//        var lastx = xpoints[npoints - 1]
//        var lasty = ypoints[npoints - 1]
        var last = vertices.last()
        var cur: Translation2d

        // Walk the edges of the polygon
        var i = 0
        while (i < vertices.size) {
            cur = vertices[i]

            if (cur.y == last.y) {
                last = cur
                i++
                continue
            }

            val leftx: Double
            if (cur.x < last.x) {
                if (point.x >= last.x) {
                    last = cur
                    i++
                    continue
                }
                leftx = cur.x
            } else {
                if (point.x >= cur.x) {
                    last = cur
                    i++
                    continue
                }
                leftx = last.x
            }

            val test1: Double
            val test2: Double
            if (cur.y < last.y) {
                if (point.y < cur.y || point.y >= last.y) {
                    last = cur
                    i++
                    continue
                }
                if (point.x < leftx) {
                    hits++
                    last = cur
                    i++
                    continue
                }
                test1 = point.x - cur.x
                test2 = point.y - cur.y
            } else {
                if (point.y < last.y || point.y >= cur.y) {
                    last = cur
                    i++
                    continue
                }
                if (point.x < leftx) {
                    hits++
                    last = cur
                    i++
                    continue
                }
                test1 = point.x - last.x
                test2 = point.y - last.y
            }

            if (test1 < (test2 / (last.y - cur.y) * (last.x - cur.x))) {
                hits++
            }
            last = cur
            i++
        }

        return ((hits and 1) != 0)
    }


}