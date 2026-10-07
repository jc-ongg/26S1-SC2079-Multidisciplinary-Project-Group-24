package com.example.mdpgroup24.bluetooth

/** Session history survives Chat view recreation and tab switches. */
class ChatHistory {
    enum class Kind { SENT, RECEIVED, SYSTEM }
    data class Entry(val kind: Kind, val message: String, val time: Long)

    private val entries = mutableListOf<Entry>()
    var revision = 0L
        private set
    fun snapshot(): List<Entry> = entries.toList()
    fun clear() { entries.clear(); revision++ }

    fun add(kind: Kind, message: String, time: Long = System.currentTimeMillis()) {
        if (message.isBlank()) return
        entries.add(Entry(kind, message, time))
        revision++
        if (entries.size > 500) entries.removeAt(0)
    }

    fun receiveFrame(payload: String) {
        val parts = payload.removePrefix("/").split(",").map { it.trim() }
        when (parts.firstOrNull()?.uppercase()) {
            "TARGET" -> {
                if (parts.size != 3) return
                val obstacle = parts[1].toIntOrNull() ?: return
                val target = parts[2].toIntOrNull() ?: return
                add(Kind.RECEIVED, "Target found: obstacle $obstacle, image ID $target")
            }
            "STATUS" -> add(Kind.RECEIVED, payload.substringAfter(",").trim())
        }
    }
}
