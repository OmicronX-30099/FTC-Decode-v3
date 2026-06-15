@file:Suppress("UNCHECKED_CAST")

import com.pedropathing.geometry.Pose
import org.yaml.snakeyaml.Yaml
import java.io.File
import java.io.FileInputStream


fun main() {
    PathManager.loadFile("test_path.yaml")
}
object PathManager {
    var startPose = Pose()
        private set

    fun loadFile(fName: String) {
        val yamlReader = Yaml()
        val pathFile = File("Teamcode/src/main/java/org/firstinspires/ftc/teamcode", fName)

        val pathData: Map<String, Any>
            = FileInputStream(pathFile).use {
                inputStream -> yamlReader.load(inputStream)
            }

        readPathData(pathData)
    }

    private fun readPathData(data: Map<String, Any>) {
        startPose = readPose(data["startPoint"] as Map<String, Any>)
        val lines = 0.0
    }
    private fun readPose(data: Map<String, Any>): Pose
        = Pose(
        (data["x"] as Number).toDouble(),
        (data["y"] as Number).toDouble()
        )
}