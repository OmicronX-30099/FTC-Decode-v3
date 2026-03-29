package org.firstinspires.ftc.teamcode.Template.Gamepad

import java.util.function.Supplier
import kotlin.properties.Delegates

class Listener(private val condition: Supplier<Boolean>): Supplier<Boolean> {
    private class TaskSheet {
        private val tasks: MutableList<Runnable> = mutableListOf<Runnable>()

        fun completeTasks() = tasks.forEach {
            it.run()
        }
        fun addTask(task: Runnable) = tasks.add(task)
        fun clearTasks() = tasks.clear()
    }

    private var state: Boolean by Delegates.notNull<Boolean>()
    private var prevState: Boolean by Delegates.notNull<Boolean>()

    override fun get(): Boolean = condition.get()

    private val risingEdgeTaskSheet: TaskSheet = TaskSheet()
    private val fallingEdgeTaskSheet: TaskSheet = TaskSheet()
    private val activeStateTaskSheet: TaskSheet = TaskSheet()
    private val inactiveStateTaskSheet: TaskSheet = TaskSheet()

    private val taskSheets: List<TaskSheet> = listOf(risingEdgeTaskSheet, fallingEdgeTaskSheet, activeStateTaskSheet, inactiveStateTaskSheet)

    infix fun whenBecomesTrue(task: Runnable) = apply { risingEdgeTaskSheet.addTask(task) }
    infix fun whenBecomesFalse(task: Runnable) = apply { fallingEdgeTaskSheet.addTask(task) }
    infix fun whileTrue(task: Runnable) = apply { activeStateTaskSheet.addTask(task) }
    infix fun whileFalse(task: Runnable) = apply { inactiveStateTaskSheet.addTask(task) }

    infix fun and(otherListener: Supplier<Boolean>): Listener = Listener ({ this.get() && otherListener.get() })
    infix fun or(otherListener: Supplier<Boolean>): Listener = Listener { this.get() || otherListener.get() }
    infix fun xor(otherListener: Supplier<Boolean>): Listener = Listener { this.get() xor otherListener.get() }
    operator fun plus(otherListener: Listener): Listener = this.and(otherListener)
    operator fun not(): Listener = Listener { !condition.get() }

    fun listen() {
        prevState = state
        state = condition.get()

        if (state) activeStateTaskSheet.completeTasks()
        if (!state) inactiveStateTaskSheet.completeTasks()
        if (state && !prevState) risingEdgeTaskSheet.completeTasks()
        if (!state && prevState) fallingEdgeTaskSheet.completeTasks()
    }

    fun reset() = taskSheets.forEach { it.clearTasks() }
}