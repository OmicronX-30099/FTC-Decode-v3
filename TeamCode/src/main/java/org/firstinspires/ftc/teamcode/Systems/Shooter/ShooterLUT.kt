package org.firstinspires.ftc.teamcode.Systems.Shooter

object ShooterLUT {

    private val table = listOf(
        70.0 to 1360.0,
        76.5 to 1400.0,
        86.5 to 1480.0,
        96.5 to 1540.0,
        106.5 to 1600.0,
        116.5 to 1680.0,
        126.5 to 1780.0,
        137.5 to 1800.0,
        147.5 to 1800.0,
        157.5 to 1920.0,
        167.5 to 1960.0
    )

    fun getVelocity(distance: Double): Double {
        val raw = interpolate(distance)
        return roundToNearest10(raw)
    }

    private fun interpolate(distance: Double): Double {
        if (distance <= table.first().first) return table.first().second
        if (distance >= table.last().first) return table.last().second

        var low = 0
        var high = table.lastIndex

        while (low <= high) {
            val mid = (low + high) / 2
            val midDistance = table[mid].first

            when {
                distance < midDistance -> high = mid - 1
                distance > midDistance -> low = mid + 1
                else -> return table[mid].second
            }
        }

        val (d1, v1) = table[high]
        val (d2, v2) = table[low]

        val t = (distance - d1) / (d2 - d1)
        return v1 + t * (v2 - v1)
    }

    private fun roundToNearest10(value: Double): Double {
        return kotlin.math.round(value / 10.0) * 10.0
    }
}