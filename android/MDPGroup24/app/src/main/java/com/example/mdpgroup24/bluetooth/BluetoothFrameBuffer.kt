package com.example.mdpgroup24.bluetooth

/** RFCOMM reads may split delimiters or contain several frames. */
class BluetoothFrameBuffer {
    private val buffer = StringBuilder()

    data class Batch(val frames: List<String>, val text: List<String>)

    fun clear() = buffer.clear()

    fun feed(chunk: String): Batch {
        buffer.append(chunk)
        val frames = mutableListOf<String>()
        val text = mutableListOf<String>()
        while (buffer.isNotEmpty()) {
            val start = buffer.indexOf("//")
            if (start < 0) {
                val keep = if (buffer.last() == '/') 1 else 0
                buffer.substring(0, buffer.length - keep).trim()
                    .takeIf { it.isNotEmpty() }?.let(text::add)
                buffer.delete(0, buffer.length - keep)
                break
            }
            if (start > 0) {
                buffer.substring(0, start).trim().takeIf { it.isNotEmpty() }?.let(text::add)
                buffer.delete(0, start)
            }
            val end = buffer.indexOf("//", 2)
            if (end < 0) break
            buffer.substring(2, end).trim().takeIf { it.isNotEmpty() }?.let(frames::add)
            buffer.delete(0, end + 2)
        }
        return Batch(frames, text)
    }
}
