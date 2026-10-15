package org.openbot.controller

import android.util.Log
import kotlin.math.absoluteValue

object DriveCommandReducer {
    private const val TAG = "DriveCommandReducer"

    private var lastRight = 0f
    private var lastLeft = 0f
    private const val withinRange = .02f

    /**
     * Optional observer for the UI. Called with the latest (right, left) values on every
     * filter() call, including changes too small to be sent. It may be called from a
     * non-UI thread.
     */
    @Volatile
    var onDriveValues: ((right: Float, left: Float) -> Unit)? = null

    fun filter(rightValue: Float, leftValue: Float) {
        onDriveValues?.invoke(rightValue, leftValue)

        if (isDifferent(rightValue, leftValue)) {
            lastLeft = leftValue
            lastRight = rightValue
            val msg = """{"driveCmd":{"r":${rightValue},"l":${leftValue}}}"""
            ConnectionSelector.getConnection().sendMessage(msg)
        }
    }

    /**
     * Explicit STOP: always sends r = 0, l = 0 and does NOT go through the change filter, so it
     * can never be suppressed as "too small a change". It also resets the filter's memory so the
     * next command is compared against zero.
     */
    fun sendExplicitStop() {
        lastLeft = 0f
        lastRight = 0f
        onDriveValues?.invoke(0f, 0f)
        try {
            ConnectionSelector.getConnection()
                .sendMessage("""{"driveCmd":{"r":0.0,"l":0.0}}""")
        } catch (e: Exception) {
            Log.w(TAG, "Could not send explicit stop: $e")
        }
    }

    private fun isDifferent (right: Float, left: Float): Boolean {
        if ((left - lastLeft).absoluteValue <= withinRange && (right - lastRight).absoluteValue <= withinRange) {
            return false
        }
        return true
    }
}