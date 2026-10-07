package com.example.mdpgroup24.bluetooth

/** A READY reply is valid only for the latest submitted map in this connection. */
class Task1Preparation {
    enum class State { UNSENT, PLANNING, READY, RUNNING, ERROR }

    var state = State.UNSENT
        private set
    var requestId: String? = null
        private set
    var error: String? = null
        private set

    val canStart: Boolean get() = state == State.READY && requestId != null

    fun submit(id: String) {
        requestId = id
        error = null
        state = State.PLANNING
    }

    fun invalidate() {
        requestId = null
        error = null
        state = State.UNSENT
    }

    fun receive(payload: String): Boolean {
        val parts = payload.split(",", limit = 4)
        if (parts.size < 3 || parts[0] != "PLAN" || parts[1] != requestId ||
            state != State.PLANNING) return false
        when (parts[2]) {
            "READY" -> state = State.READY
            "ERROR" -> {
                state = State.ERROR
                error = parts.getOrNull(3) ?: "Route preparation failed"
            }
            else -> return false
        }
        return true
    }

    fun start(): Boolean {
        if (!canStart) return false
        state = State.RUNNING
        return true
    }
}
