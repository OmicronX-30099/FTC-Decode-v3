package org.firstinspires.ftc.teamcode.Systems.Shooter

import kotlin.math.pow
import kotlin.math.round
import kotlin.math.sqrt

object ShooterLUT {

    data class Sample(
        val x: Double,
        val y: Double,
        val velocity: Double
    )

    private data class WeightedSample(
        val sample: Sample,
        val distance: Double
    )

    private val table = listOf(
        Sample(44.0, 99.0, 1250.0),
        Sample(68.0, 120.0, 1300.0),
        Sample(78.0, 119.0, 1340.0),
        Sample(89.0, 118.0, 1400.0),
        Sample(99.0, 116.0, 1500.0),
        Sample(112.0, 114.0, 1580.0),
        Sample(92.0, 92.0, 1580.0),
        Sample(80.0, 94.0, 1500.0),
        Sample(69.0, 95.0, 1420.0),
        Sample(59.0, 102.0, 1300.0),
        Sample(54.0, 87.0, 1360.0),
        Sample(66.0, 86.0, 1420.0),
        Sample(62.0, 76.0, 1480.0),
        Sample(59.0, 10.0, 1800.0),
        Sample(50.0, 10.0, 1760.0),
        Sample(41.0, 10.0, 1740.0),
        Sample(73.0, 10.0, 1900.0),
        Sample(83.0, 10.0, 1940.0),
        Sample(93.0, 10.0, 2000.0)
    )

    private const val POWER = 2.0
    private const val EPSILON = 1e-6
    private const val NEIGHBOR_COUNT = 4

    fun getVelocity(x: Double, y: Double): Double {
        val raw = interpolateIDW(x, y)
        return roundToNearest10(raw)
    }

    private fun interpolateIDW(x: Double, y: Double): Double {
        val nearby = table
            .map { sample ->
                val dx = x - sample.x
                val dy = y - sample.y
                WeightedSample(sample, sqrt(dx * dx + dy * dy))
            }
            .sortedBy { it.distance }
            .take(NEIGHBOR_COUNT)

        if (nearby.isEmpty()) return 0.0

        if (nearby.first().distance < EPSILON) {
            return nearby.first().sample.velocity
        }

        var weightedSum = 0.0
        var weightSum = 0.0

        for (entry in nearby) {
            val weight = 1.0 / entry.distance.pow(POWER)
            weightedSum += entry.sample.velocity * weight
            weightSum += weight
        }

        return weightedSum / weightSum
    }

    private fun roundToNearest10(value: Double): Double {
        return round(value / 10.0) * 10.0
    }
}