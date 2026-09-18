package com.example.mdpgroup24.bluetooth

import android.bluetooth.BluetoothDevice
import android.content.Context

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
    }


    private lateinit var connectionService:
            BluetoothConnectionService


    private val listeners =
        mutableSetOf<Listener>()

    private var currentState =
        BluetoothConnectionService.ConnectionState.DISCONNECTED


    private var currentDevice:
            BluetoothDevice? = null


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

        listeners
            .toList()
            .forEach { listener ->

                listener
                    .onMessageReceived(
                        message
                    )
            }
    }
}