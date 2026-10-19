package com.example.mdpgroup24.bluetooth

import android.bluetooth.BluetoothDevice
import android.content.Context
import java.util.UUID
import android.os.SystemClock
import android.os.Handler
import android.os.Looper

object BluetoothSession :
    BluetoothConnectionService.Listener {

    interface Listener {

        fun onConnectionStateChanged(
            state: BluetoothConnectionService.ConnectionState,
            device: BluetoothDevice?
        )

        fun onMessageReceived(
            message: String
        )

        fun onPreparationChanged() {}
        fun onChatHistoryChanged() {}
        fun onTaskStatus(status: String) {}
        fun onManualCompleted(command: String) {}
        fun onTargetsChanged() {}
    }


    private lateinit var connectionService:
            BluetoothConnectionService


    private val listeners =
        mutableSetOf<Listener>()

    private var currentState =
        BluetoothConnectionService.ConnectionState.DISCONNECTED


    private var currentDevice:
            BluetoothDevice? = null

    val preparation = Task1Preparation()
    val chatHistory = ChatHistory()
    private val frames = BluetoothFrameBuffer()
    private val recognizedTargets = mutableMapOf<Int, Int>()
    fun targetSnapshot(): Map<Int, Int> = recognizedTargets.toMap()
    fun clearTargets() {
        recognizedTargets.clear()
        listeners.toList().forEach { it.onTargetsChanged() }
    }
    private var pendingManual: String? = null
    var task1StartedAt: Long? = null
        private set
    var task1FinishedAt: Long? = null
        private set
    val task1Running: Boolean get() = task1StartedAt != null && task1FinishedAt == null

    fun resetTask1Timer() { task1StartedAt = null; task1FinishedAt = null }
    fun finishTask1Timer() {
        if (task1Running) task1FinishedAt = SystemClock.elapsedRealtime()
    }

    fun sendManual(command: String): Boolean {
        if (!isConnected || task1Running || pendingManual != null) return false
        invalidatePreparation()
        if (!send("//$command//")) return false
        pendingManual = command
        return true
    }

    fun invalidatePreparation() {
        preparation.invalidate()
        listeners.toList().forEach { it.onPreparationChanged() }
    }

    fun sendMap(map: String): Boolean {
        if (!isConnected || task1Running || pendingManual != null) return false
        val id = UUID.randomUUID().toString().replace("-", "")
        preparation.submit(id)
        listeners.toList().forEach { it.onPreparationChanged() }
        val payload = map.replaceFirst("MAP", "MAP,$id")
        if (!send("//$payload//")) {
            invalidatePreparation()
            return false
        }
        clearTargets()
        recordChat(ChatHistory.Kind.SENT, "Map sent. Preparing Task 1 route.")
        // Pi planning can take 180 s, plus connection and reply delivery time.
        Handler(Looper.getMainLooper()).postDelayed({
            if (preparation.requestId == id && preparation.state == Task1Preparation.State.PLANNING) {
                preparation.receive("PLAN,$id,ERROR,No route confirmation. Check the Pi update and Algo connection; send map again.")
                listeners.toList().forEach { it.onPreparationChanged() }
                recordChat(ChatHistory.Kind.SYSTEM, preparation.error ?: "Route preparation timed out")
            }
        }, 190_000L)
        return true
    }

    fun startTask1(): Boolean {
        if (!isConnected || task1Running || pendingManual != null || !preparation.canStart) return false
        if (!send("//BEGIN,1,${preparation.requestId}//")) return false
        task1StartedAt = SystemClock.elapsedRealtime()
        task1FinishedAt = null
        preparation.start()
        listeners.toList().forEach { it.onPreparationChanged() }
        recordChat(ChatHistory.Kind.SENT, "Task 1 started")
        return true
    }

    fun recordChat(kind: ChatHistory.Kind, message: String) {
        chatHistory.add(kind, message)
        listeners.toList().forEach { it.onChatHistoryChanged() }
    }

    fun clearChat() {
        chatHistory.clear()
        listeners.toList().forEach { it.onChatHistoryChanged() }
    }


    // ============================================================
    // INITIALISE
    // ============================================================

    fun initialize(
        context: Context
    ) {

        if (::connectionService.isInitialized) {
            return
        }


        connectionService =
            BluetoothConnectionService(
                context.applicationContext,
                this
            )


        connectionService.startListening()
    }

    fun ensureListening() {

        if (!::connectionService.isInitialized) {
            return
        }


        connectionService.startListening()
    }


    // ============================================================
    // LISTENERS
    // ============================================================

    fun addListener(
        listener: Listener
    ) {

        listeners.add(
            listener
        )


        listener.onConnectionStateChanged(
            currentState,
            currentDevice
        )
        listener.onTargetsChanged()
    }


    fun removeListener(
        listener: Listener
    ) {

        listeners.remove(
            listener
        )
    }


    // ============================================================
    // CONNECTION
    // ============================================================

    fun connect(
        device: BluetoothDevice
    ) {

        if (
            !::connectionService
                .isInitialized
        ) {
            return
        }


        connectionService.connect(
            device
        )
    }


    fun disconnect() {

        if (
            !::connectionService
                .isInitialized
        ) {
            return
        }


        connectionService.disconnect()
    }


    val isConnected: Boolean

        get() =

            ::connectionService
                .isInitialized &&
                    connectionService
                        .isConnected


    // ============================================================
    // SEND
    // ============================================================

    fun send(
        message: String
    ): Boolean {

        if (
            !::connectionService
                .isInitialized
        ) {
            return false
        }


        return connectionService.write(
            message
        )
    }


    // ============================================================
    // CALLBACKS
    // ============================================================

    override fun onConnectionStateChanged(
        state: BluetoothConnectionService.ConnectionState,
        device: BluetoothDevice?
    ) {

        frames.clear()
        pendingManual = null
        invalidatePreparation()
        currentState =
            state


        currentDevice =
            when (state) {

                BluetoothConnectionService.ConnectionState.CONNECTING,
                BluetoothConnectionService.ConnectionState.CONNECTED -> {
                    device
                }

                BluetoothConnectionService.ConnectionState.DISCONNECTED,
                BluetoothConnectionService.ConnectionState.FAILED -> {
                    null
                }
            }


        listeners
            .toList()
            .forEach { listener ->

                listener
                    .onConnectionStateChanged(
                        state,
                        currentDevice
                    )
            }
    }


    override fun onMessageReceived(
        message: String
    ) {

        // Apply TARGET/ROBOT to the visible map before handling completion.
        listeners.toList().forEach { it.onMessageReceived(message) }

        val previousHistory = chatHistory.revision
        val batch = frames.feed(message)
        batch.text.forEach { chatHistory.add(ChatHistory.Kind.RECEIVED, it) }
        batch.frames.forEach { payload ->
            if (payload.startsWith("TARGET,")) {
                val parts = payload.split(",").map { it.trim() }
                val obstacle = parts.getOrNull(1)?.toIntOrNull()
                val target = parts.getOrNull(2)?.toIntOrNull()
                if (parts.size == 3 && obstacle != null && obstacle > 0 && target != null && target in 11..40) {
                    if (recognizedTargets[obstacle] != target) {
                        recognizedTargets[obstacle] = target
                        chatHistory.receiveFrame(payload)
                        listeners.toList().forEach { it.onTargetsChanged() }
                    }
                }
            } else {
                chatHistory.receiveFrame(payload)
            }
            if (preparation.receive(payload)) {
                listeners.toList().forEach { it.onPreparationChanged() }
                if (preparation.canStart) {
                    chatHistory.add(ChatHistory.Kind.RECEIVED, "Task 1 route ready. You can press Start.")
                } else if (preparation.state == Task1Preparation.State.ERROR) {
                    chatHistory.add(ChatHistory.Kind.SYSTEM, preparation.error ?: "Route preparation failed")
                }
            }
            if (payload.startsWith("STATUS,")) {
                val status = payload.substringAfter(",").trim()
                val command = pendingManual
                val name = mapOf("F" to "forward", "B" to "backward", "L" to "left", "R" to "right")[command]
                if (command != null && status == "Manual $name complete") {
                    pendingManual = null
                    listeners.toList().forEach { it.onManualCompleted(command) }
                }
                if (status.contains("paused", true) || status.contains("failed", true) || status.contains("busy", true)) {
                    pendingManual = null
                }
                if (status.equals("finished", true)) finishTask1Timer()
                listeners.toList().forEach { it.onTaskStatus(status) }
            }
        }
        if (chatHistory.revision != previousHistory) {
            listeners.toList().forEach { it.onChatHistoryChanged() }
        }

    }
}
