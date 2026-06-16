@file:Suppress("UNCHECKED_CAST", "SameParameterValue", "PackageDirectoryMismatch")

import android.R.attr.data
import android.R.attr.path
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
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

        for (pChain in pathChains) {
            parsePathChain(pChain)
        }
    }

    private fun parsePathChain(data: Map<String, Any>)/*: PathChain*/ {
        //val pChainStart: PathBuilder = follower.pathBuilder()
        val paths = data["paths"] as ArrayList<Map<String, Any>>
        for (path in paths) {
            applyPath(path)
        }

        //return pChainStart.build()
    }

    private fun applyPath(data: Map<String, Any>, /*currPChain: PathBuilder*/)/*: PathBuilder*/ {
        // Start point
        val startPoint = parsePose(data["startPoint"] as Map<String, Any>)
        val controlPoints: ArrayList<Pose> = arrayListOf(startPoint)

        // Control Points
        val controlPointData = data["controlPoints"] as ArrayList<Map<String, Any>>
        for (point in controlPointData) { controlPoints.add(parsePose(point)) }

        // End point
        val endPoint = parsePose(data["endPoint"] as Map<String, Any>)
        controlPoints.add(endPoint)
        /*
        // Apply to builder
        when (controlPoints.size) {
            2 -> {
                currPChain.addPath(
                    BezierLine(startPoint, endPoint)
                )
            }
            else -> {
                currPChain.addPath(
                    BezierCurve(controlPoints)
                )
            }
        }*/
        println(controlPoints)
        //return currPChain
    }

    private fun parsePose(data: Map<String, Any>)
        = Pose(
            roundTo((data["x"] as Number).toDouble(), 5),
            roundTo((data["y"] as Number).toDouble(), 5),
        )
    private fun roundTo(num: Double, digits: Int): Double
        = round(num * 10.0.pow(digits)) / 10.0.pow(digits)

}