package com.example.mdpgroup24.bluetooth

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothServerSocket
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.core.content.ContextCompat
import java.io.InputStream
import java.io.OutputStream
import java.util.UUID

class BluetoothConnectionService(
    context: Context,
    private val listener: Listener
) {

    interface Listener {

        fun onConnectionStateChanged(
            state: ConnectionState,
            device: BluetoothDevice?
        )

        fun onMessageReceived(
            message: String
        )
    }


    enum class ConnectionState {
        CONNECTING,
        CONNECTED,
        DISCONNECTED,
        FAILED
    }


    companion object {

        private const val TAG =
            "BluetoothConnection"

        private const val SERVICE_NAME =
            "MDP_Grp_24"

        private const val LISTEN_RETRY_DELAY_MS =
            2000L

        val SPP_UUID: UUID =
            UUID.fromString(
                "00001101-0000-1000-8000-00805F9B34FB"
            )
    }


    private val context =
        context.applicationContext


    private val bluetoothAdapter: BluetoothAdapter? =
        context
            .getSystemService(
                BluetoothManager::class.java
            )
            .adapter


    private val mainHandler =
        Handler(
            Looper.getMainLooper()
        )


    private var acceptThread:
            AcceptThread? = null

    private var connectThread:
            ConnectThread? = null

    private var connectedThread:
            ConnectedThread? = null


    private var listenRetryRunnable:
            Runnable? = null


    @Volatile
    private var destroyed =
        false


    @Volatile
    var isConnected: Boolean =
        false
        private set


    // ============================================================
    // SERVER / LISTENING MODE
    // ============================================================

    @SuppressLint("MissingPermission")
    @Synchronized
    fun startListening() {

        if (destroyed) {
            return
        }


        if (isConnected) {
            return
        }


        if (!hasConnectPermission()) {

            Log.d(
                TAG,
                "Cannot start listener: BLUETOOTH_CONNECT missing"
            )

            return
        }


        val adapter =
            bluetoothAdapter
                ?: return


        if (!adapter.isEnabled) {

            Log.d(
                TAG,
                "Bluetooth off. Listener retry scheduled."
            )

            scheduleListeningRetry()

            return
        }


        if (
            acceptThread?.isAlive ==
            true
        ) {
            return
        }


        cancelListeningRetry()


        val thread =
            AcceptThread()


        if (!thread.isValid()) {

            Log.e(
                TAG,
                "RFCOMM server socket could not be created"
            )

            scheduleListeningRetry()

            return
        }


        acceptThread =
            thread


        Log.d(
            TAG,
            "Starting RFCOMM server listener"
        )


        thread.start()
    }


    // ============================================================
    // ACCEPT THREAD
    // ============================================================

    @SuppressLint("MissingPermission")
    private inner class AcceptThread :
        Thread(
            "BluetoothAcceptThread"
        ) {

        @Volatile
        private var cancelled =
            false


        private val serverSocket:
                BluetoothServerSocket? =

            try {

                /*
                 * Pante's implementation uses the insecure
                 * RFCOMM server form for incoming AMDTool
                 * connections.
                 */
                bluetoothAdapter
                    ?.listenUsingInsecureRfcommWithServiceRecord(
                        SERVICE_NAME,
                        SPP_UUID
                    )

            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "Failed to create RFCOMM server socket",
                    e
                )

                null
            }


        fun isValid():
                Boolean {

            return serverSocket != null
        }


        override fun run() {

            Log.d(
                TAG,
                "Waiting for incoming RFCOMM connection"
            )


            try {

                val socket =
                    serverSocket
                        ?.accept()


                if (
                    socket != null &&
                    !cancelled
                ) {

                    val device =
                        socket.remoteDevice


                    Log.d(
                        TAG,
                        "Incoming connection from ${device.address}"
                    )


                    incomingConnectionEstablished(
                        this,
                        socket,
                        device
                    )

                } else {

                    try {
                        socket?.close()
                    } catch (_: Exception) {
                    }
                }


            } catch (e: Exception) {

                if (!cancelled) {

                    Log.e(
                        TAG,
                        "RFCOMM accept failed",
                        e
                    )
                }

            } finally {

                acceptThreadEnded(
                    this,
                    cancelled
                )
            }
        }


        fun cancel() {

            cancelled =
                true


            interrupt()


            try {

                serverSocket
                    ?.close()

            } catch (_: Exception) {
            }
        }
    }


    // ============================================================
    // ACCEPT THREAD FINISHED
    // ============================================================

    private fun acceptThreadEnded(
        sourceThread: AcceptThread,
        wasCancelled: Boolean
    ) {

        var restart =
            false


        synchronized(this) {

            if (
                acceptThread ===
                sourceThread
            ) {

                acceptThread =
                    null


                restart =
                    !wasCancelled &&
                            !destroyed &&
                            !isConnected
            }
        }


        if (restart) {

            scheduleListeningRetry()
        }
    }


    // ============================================================
    // INCOMING CONNECTION ESTABLISHED
    // ============================================================

    private fun incomingConnectionEstablished(
        sourceThread: AcceptThread,
        socket: BluetoothSocket,
        device: BluetoothDevice
    ) {

        var accepted =
            false


        synchronized(this) {

            /*
             * Only the currently registered AcceptThread
             * is allowed to establish a connection.
             */
            if (
                acceptThread !==
                sourceThread ||
                destroyed
            ) {

                try {
                    socket.close()
                } catch (_: Exception) {
                }

                return
            }


            /*
             * If another connection won the race first,
             * discard this socket.
             */
            if (isConnected) {

                try {
                    socket.close()
                } catch (_: Exception) {
                }

                return
            }


            /*
             * Incoming connection wins over an outgoing
             * attempt currently in progress.
             */
            val oldConnectThread =
                connectThread


            connectThread =
                null


            oldConnectThread
                ?.cancel()


            /*
             * Stop accepting more sockets while connected.
             */
            acceptThread =
                null


            sourceThread.cancel()


            val newThread =
                ConnectedThread(
                    socket,
                    device
                )


            connectedThread =
                newThread


            isConnected =
                true


            cancelListeningRetry()


            newThread.start()


            accepted =
                true
        }


        if (accepted) {

            notifyConnectionState(
                ConnectionState.CONNECTED,
                device
            )
        }
    }


    // ============================================================
    // OUTGOING CONNECTION
    // ============================================================

    @Synchronized
    fun connect(
        device: BluetoothDevice
    ) {

        if (
            destroyed ||
            !hasConnectPermission()
        ) {
            return
        }


        // =====================================================
        // PREVENT DUPLICATE CONNECTION ATTEMPTS
        // =====================================================

        if (isConnected) {

            Log.d(
                TAG,
                "Ignoring connection request: already connected"
            )

            return
        }


        if (
            connectThread?.isAlive ==
            true
        ) {

            Log.d(
                TAG,
                "Ignoring connection request: already connecting"
            )

            return
        }


        Log.d(
            TAG,
            "Outgoing connection requested to ${device.address}"
        )


        /*
         * Leave AcceptThread alive.
         *
         * This allows either side to establish the
         * connection. Whichever succeeds first wins.
         */
        startListening()


        /*
         * Cancel previous outgoing attempt.
         */
        val oldConnectThread =
            connectThread


        connectThread =
            null


        oldConnectThread
            ?.cancel()


        /*
         * Close existing connection if user intentionally
         * selects/connects another device.
         */
        val oldConnectedThread =
            connectedThread


        connectedThread =
            null


        isConnected =
            false


        oldConnectedThread
            ?.cancel()


        notifyConnectionState(
            ConnectionState.CONNECTING,
            device
        )


        val thread =
            ConnectThread(
                device
            )


        connectThread =
            thread


        thread.start()
    }


    // ============================================================
    // CONNECT THREAD
    // ============================================================

    @SuppressLint("MissingPermission")
    private inner class ConnectThread(
        private val device: BluetoothDevice
    ) : Thread(
        "BluetoothConnectThread"
    ) {

        @Volatile
        private var cancelled =
            false


        private var socket:
                BluetoothSocket? = null


        override fun run() {

            stopDiscovery()


            if (cancelled) {
                return
            }


            val candidateSocket =
                try {

                    /*
                     * Your original outgoing connection works
                     * using the standard secure RFCOMM method.
                     */
                    device
                        .createRfcommSocketToServiceRecord(
                            SPP_UUID
                        )

                } catch (e: Exception) {

                    Log.e(
                        TAG,
                        "Unable to create outgoing RFCOMM socket",
                        e
                    )

                    null
                }


            if (candidateSocket == null) {

                outgoingConnectionFailed(
                    this,
                    device
                )

                return
            }


            socket =
                candidateSocket


            try {

                Log.d(
                    TAG,
                    "Connecting to ${device.address}"
                )


                candidateSocket.connect()


                if (cancelled) {

                    try {
                        candidateSocket.close()
                    } catch (_: Exception) {
                    }

                    return
                }


                Log.d(
                    TAG,
                    "Outgoing RFCOMM connection established"
                )


                outgoingConnectionEstablished(
                    this,
                    candidateSocket,
                    device
                )


            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "Outgoing RFCOMM connection failed",
                    e
                )


                try {
                    candidateSocket.close()
                } catch (_: Exception) {
                }


                if (!cancelled) {

                    outgoingConnectionFailed(
                        this,
                        device
                    )
                }
            }
        }


        fun cancel() {

            cancelled =
                true


            interrupt()


            try {

                socket
                    ?.close()

            } catch (_: Exception) {
            }


            socket =
                null
        }
    }


    // ============================================================
    // OUTGOING CONNECTION SUCCESS
    // ============================================================

    private fun outgoingConnectionEstablished(
        sourceThread: ConnectThread,
        socket: BluetoothSocket,
        device: BluetoothDevice
    ) {

        var accepted =
            false


        synchronized(this) {

            if (
                connectThread !==
                sourceThread ||
                destroyed
            ) {

                try {
                    socket.close()
                } catch (_: Exception) {
                }

                return
            }


            /*
             * Incoming connection may have completed
             * immediately before this one.
             */
            if (isConnected) {

                connectThread =
                    null


                try {
                    socket.close()
                } catch (_: Exception) {
                }

                return
            }


            connectThread =
                null


            /*
             * We have one active socket now.
             * Stop the server until it disconnects.
             */
            val oldAcceptThread =
                acceptThread


            acceptThread =
                null


            oldAcceptThread
                ?.cancel()


            val newThread =
                ConnectedThread(
                    socket,
                    device
                )


            connectedThread =
                newThread


            isConnected =
                true


            cancelListeningRetry()


            newThread.start()


            accepted =
                true
        }


        if (accepted) {

            notifyConnectionState(
                ConnectionState.CONNECTED,
                device
            )
        }
    }


    // ============================================================
    // OUTGOING CONNECTION FAILURE
    // ============================================================

    private fun outgoingConnectionFailed(
        sourceThread: ConnectThread,
        device: BluetoothDevice
    ) {

        var notifyFailure =
            false


        synchronized(this) {

            if (
                connectThread !==
                sourceThread
            ) {
                return
            }


            connectThread =
                null


            if (!isConnected) {

                notifyFailure =
                    true
            }
        }


        if (notifyFailure) {

            notifyConnectionState(
                ConnectionState.FAILED,
                device
            )


            /*
             * Important:
             * an outgoing connection failure must NOT
             * disable incoming AMDTool connections.
             */
            startListening()
        }
    }


    // ============================================================
    // ACTIVE CONNECTION
    // ============================================================

    private inner class ConnectedThread(
        private val socket: BluetoothSocket,
        private val device: BluetoothDevice
    ) : Thread(
        "BluetoothConnectedThread"
    ) {

        private val inputStream:
                InputStream? =

            try {

                socket.inputStream

            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "Unable to obtain Bluetooth input stream",
                    e
                )

                null
            }


        private val outputStream:
                OutputStream? =

            try {

                socket.outputStream

            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "Unable to obtain Bluetooth output stream",
                    e
                )

                null
            }


        override fun run() {

            val input =
                inputStream


            if (input == null) {

                connectionLost(
                    this,
                    device
                )

                return
            }


            val buffer =
                ByteArray(
                    1024
                )


            while (!isInterrupted) {

                try {

                    val bytesRead =
                        input.read(
                            buffer
                        )


                    if (bytesRead <= 0) {
                        break
                    }


                    val message =
                        String(
                            buffer,
                            0,
                            bytesRead,
                            Charsets.UTF_8
                        )


                    Log.d(
                        TAG,
                        "Received: $message"
                    )


                    mainHandler.post {

                        listener
                            .onMessageReceived(
                                message
                            )
                    }


                } catch (e: Exception) {

                    Log.d(
                        TAG,
                        "Bluetooth read ended: ${e.message}"
                    )

                    break
                }
            }


            connectionLost(
                this,
                device
            )
        }


        fun write(
            message: String
        ): Boolean {

            val output =
                outputStream
                    ?: return false


            return try {

                output.write(
                    message
                        .toByteArray(
                            Charsets.UTF_8
                        )
                )


                output.flush()


                Log.d(
                    TAG,
                    "Sent: $message"
                )


                true

            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "Bluetooth write failed",
                    e
                )


                connectionLost(
                    this,
                    device
                )


                false
            }
        }


        fun cancel() {

            interrupt()


            try {

                socket.close()

            } catch (_: Exception) {
            }
        }
    }


    // ============================================================
    // CONNECTION LOST
    // ============================================================

    private fun connectionLost(
        sourceThread: ConnectedThread,
        device: BluetoothDevice
    ) {

        var genuineLoss =
            false


        synchronized(this) {

            /*
             * Ignore callbacks from an old socket that
             * was intentionally replaced.
             */
            if (
                connectedThread !==
                sourceThread
            ) {
                return
            }


            connectedThread =
                null


            isConnected =
                false


            genuineLoss =
                true
        }


        sourceThread.cancel()


        if (genuineLoss) {

            Log.d(
                TAG,
                "Connection lost to ${device.address}"
            )


            notifyConnectionState(
                ConnectionState.DISCONNECTED,
                device
            )


            /*
             * C.8:
             * immediately become available for AMDTool's
             * "Reconnect to last device".
             */
            startListening()
        }
    }


    // ============================================================
    // WRITE
    // ============================================================

    fun write(
        message: String
    ): Boolean {

        val thread =
            synchronized(this) {

                if (!isConnected) {
                    return false
                }


                connectedThread
            }
                ?: return false


        return thread.write(
            message
        )
    }


    // ============================================================
    // MANUAL DISCONNECT
    // ============================================================

    @Synchronized
    fun disconnect() {

        val oldConnectThread =
            connectThread


        connectThread =
            null


        oldConnectThread
            ?.cancel()


        val oldConnectedThread =
            connectedThread


        connectedThread =
            null


        isConnected =
            false


        oldConnectedThread
            ?.cancel()


        notifyConnectionState(
            ConnectionState.DISCONNECTED,
            null
        )


        /*
         * Even after an Android-side disconnect,
         * keep the application ready to accept a
         * future AMDTool connection.
         */
        startListening()
    }


    // ============================================================
    // LISTENER RETRY
    // ============================================================

    @Synchronized
    private fun scheduleListeningRetry() {

        if (
            destroyed ||
            isConnected ||
            listenRetryRunnable != null
        ) {
            return
        }


        val runnable =
            Runnable {

                synchronized(this) {

                    listenRetryRunnable =
                        null
                }


                startListening()
            }


        listenRetryRunnable =
            runnable


        mainHandler.postDelayed(
            runnable,
            LISTEN_RETRY_DELAY_MS
        )
    }


    @Synchronized
    private fun cancelListeningRetry() {

        val runnable =
            listenRetryRunnable
                ?: return


        mainHandler.removeCallbacks(
            runnable
        )


        listenRetryRunnable =
            null
    }


    // ============================================================
    // DISCOVERY
    // ============================================================

    @SuppressLint("MissingPermission")
    private fun stopDiscovery() {

        val adapter =
            bluetoothAdapter
                ?: return


        if (hasScanPermission()) {

            try {

                adapter.cancelDiscovery()

            } catch (_: Exception) {
            }
        }
    }


    // ============================================================
    // CALLBACK
    // ============================================================

    private fun notifyConnectionState(
        state: ConnectionState,
        device: BluetoothDevice?
    ) {

        mainHandler.post {

            listener
                .onConnectionStateChanged(
                    state,
                    device
                )
        }
    }


    // ============================================================
    // PERMISSIONS
    // ============================================================

    private fun hasConnectPermission():
            Boolean {

        if (
            Build.VERSION.SDK_INT <
            Build.VERSION_CODES.S
        ) {
            return true
        }


        return ContextCompat
            .checkSelfPermission(
                context,
                Manifest.permission.BLUETOOTH_CONNECT
            ) ==
                PackageManager.PERMISSION_GRANTED
    }


    private fun hasScanPermission():
            Boolean {

        if (
            Build.VERSION.SDK_INT <
            Build.VERSION_CODES.S
        ) {
            return true
        }


        return ContextCompat
            .checkSelfPermission(
                context,
                Manifest.permission.BLUETOOTH_SCAN
            ) ==
                PackageManager.PERMISSION_GRANTED
    }


    // ============================================================
    // CLEANUP
    // ============================================================

    @Synchronized
    fun destroy() {

        destroyed =
            true


        cancelListeningRetry()


        val oldAcceptThread =
            acceptThread


        acceptThread =
            null


        oldAcceptThread
            ?.cancel()


        val oldConnectThread =
            connectThread


        connectThread =
            null


        oldConnectThread
            ?.cancel()


        val oldConnectedThread =
            connectedThread


        connectedThread =
            null


        oldConnectedThread
            ?.cancel()


        isConnected =
            false
    }
}