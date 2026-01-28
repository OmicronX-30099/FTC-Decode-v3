@file:JvmName("RGBUtil")

package org.firstinspires.ftc.teamcode.Enums

enum class RGBColor {
    AZURE { override val position: Double = 0.585 },
    BLACK { override val position: Double = 0.0 },
    BLUE { override val position: Double = 0.63 },
    GREEN { override val position: Double = 0.505 },
    RED { override val position: Double = 0.28 },
    INDIGO { override val position: Double = 0.675 },
    LIGHT_GREEN { override val position: Double = 0.465 },
    ORANGE { override val position: Double = 0.32 },
    VIOLET { override val position: Double = 0.71 },
    WHITE { override val position: Double = 1.0 },
    YELLOW { override val position: Double = 0.39 };

    abstract val position: Double
}