@file:Suppress("UNCHECKED_CAST", "SameParameterValue", "PackageDirectoryMismatch")

import android.R.attr.path
import com.pedropathing.geometry.Pose
import org.yaml.snakeyaml.Yaml
import java.io.File
import java.io.FileInputStream
import kotlin.math.pow
import kotlin.math.round

fun main() {
    AutoManager.loadFile("format.yaml")
}

object AutoManager {
    var startPoint: Pose = Pose()

    fun loadFile(fileName: String) {
        val yamlReader = Yaml()
        val pathFile = File("Teamcode/src/main/java/org/firstinspires/ftc/teamcode", fileName)

        val data: Map<String, Any> = FileInputStream(pathFile).use {
            inputStream -> yamlReader.load(inputStream)
        }

        parsePathData(data)
    }

    private fun parsePathData(data: Map<String, Any>) {
        startPoint = parsePose(data["startData"] as Map<String, Any>)
        println("The Start Pose is $startPoint")

        val pathChains = data["pathChains"] as ArrayList<Map<String, Any>>

        for (p in pathChains) {
            println(p["name"] as String)
            val paths = p["paths"] as ArrayList<Map<String, Any>>
            for (path in paths) {
                println(path)
            }
        }
    }

    private fun parsePose(data: Map<String, Any>)
        = Pose(
            roundTo((data["x"] as Number).toDouble(), 5),
            roundTo((data["y"] as Number).toDouble(), 5),
            Math.toRadians(roundTo((data["heading"] as Number).toDouble(), 5))
        )
    private fun roundTo(num: Double, digits: Int): Double
        = round(num * 10.0.pow(digits)) / 10.0.pow(digits)

}