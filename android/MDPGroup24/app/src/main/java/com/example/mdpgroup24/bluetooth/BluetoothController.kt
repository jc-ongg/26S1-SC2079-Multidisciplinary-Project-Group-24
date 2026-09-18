package com.example.mdpgroup24.bluetooth

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import androidx.core.content.ContextCompat

class BluetoothController(
    context: Context,
    private val listener: Listener
) {

    interface Listener {
        fun onDeviceFound(device: BluetoothDevice)
        fun onDiscoveryFinished(foundAny: Boolean)
        fun onDevicePaired(device: BluetoothDevice)
    }

    private val context =
        context.applicationContext

    private val bluetoothAdapter: BluetoothAdapter? =
        context
            .getSystemService(BluetoothManager::class.java)
            .adapter

    private val foundAddresses =
        mutableSetOf<String>()

    private var receiverRegistered =
        false


    // ============================================================
    // STATE
    // ============================================================

    fun isAvailable(): Boolean {
        return bluetoothAdapter != null
    }


    @SuppressLint("MissingPermission")
    fun isEnabled(): Boolean {

        if (!hasConnectPermission()) {
            return false
        }

        return bluetoothAdapter?.isEnabled == true
    }


    // ============================================================
    // PAIRED DEVICES
    // ============================================================

    @SuppressLint("MissingPermission")
    fun getPairedDevices(): List<BluetoothDevice> {

        if (!hasConnectPermission()) {
            return emptyList()
        }

        return bluetoothAdapter
            ?.bondedDevices
            ?.sortedBy { it.name ?: "" }
            ?: emptyList()
    }


    // ============================================================
    // RECEIVER
    // ============================================================

    fun registerReceiver() {

        if (receiverRegistered) {
            return
        }

        val filter = IntentFilter().apply {

            addAction(
                BluetoothDevice.ACTION_FOUND
            )

            addAction(
                BluetoothAdapter.ACTION_DISCOVERY_FINISHED
            )

            addAction(
                BluetoothDevice.ACTION_BOND_STATE_CHANGED
            )
        }


        ContextCompat.registerReceiver(
            context,
            receiver,
            filter,
            ContextCompat.RECEIVER_EXPORTED
        )

        receiverRegistered = true
    }


    private val receiver =
        object : BroadcastReceiver() {

            @SuppressLint("MissingPermission")
            override fun onReceive(
                context: Context?,
                intent: Intent?
            ) {

                when (intent?.action) {

                    BluetoothDevice.ACTION_FOUND -> {

                        if (!hasConnectPermission()) {
                            return
                        }

                        val device =
                            getBluetoothDevice(intent)
                                ?: return


                        // Already-paired devices are shown
                        // in the paired-device section.
                        if (
                            device.bondState ==
                            BluetoothDevice.BOND_BONDED
                        ) {
                            return
                        }


                        if (
                            foundAddresses.add(
                                device.address
                            )
                        ) {

                            listener.onDeviceFound(
                                device
                            )
                        }
                    }


                    BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {

                        listener.onDiscoveryFinished(
                            foundAddresses.isNotEmpty()
                        )
                    }


                    BluetoothDevice.ACTION_BOND_STATE_CHANGED -> {

                        if (!hasConnectPermission()) {
                            return
                        }

                        val device =
                            getBluetoothDevice(intent)
                                ?: return


                        if (
                            device.bondState ==
                            BluetoothDevice.BOND_BONDED
                        ) {

                            listener.onDevicePaired(
                                device
                            )
                        }
                    }
                }
            }
        }


    // ============================================================
    // DISCOVERY
    // ============================================================

    @SuppressLint("MissingPermission")
    fun startDiscovery(): Boolean {

        val adapter =
            bluetoothAdapter
                ?: return false


        if (
            !hasConnectPermission() ||
            !hasScanPermission() ||
            !adapter.isEnabled
        ) {
            return false
        }


        foundAddresses.clear()


        if (adapter.isDiscovering) {
            adapter.cancelDiscovery()
        }


        return adapter.startDiscovery()
    }


    @SuppressLint("MissingPermission")
    fun stopDiscovery() {

        val adapter =
            bluetoothAdapter
                ?: return


        if (
            hasScanPermission() &&
            adapter.isDiscovering
        ) {
            adapter.cancelDiscovery()
        }
    }


    // ============================================================
    // PAIRING
    // ============================================================

    @SuppressLint("MissingPermission")
    fun pairDevice(
        device: BluetoothDevice
    ): Boolean {

        if (!hasConnectPermission()) {
            return false
        }


        stopDiscovery()


        return try {

            device.createBond()

        } catch (_: Exception) {

            false
        }
    }


    // ============================================================
    // DEVICE EXTRACTION
    // ============================================================

    @Suppress("DEPRECATION")
    private fun getBluetoothDevice(
        intent: Intent
    ): BluetoothDevice? {

        return if (
            Build.VERSION.SDK_INT >=
            Build.VERSION_CODES.TIRAMISU
        ) {

            intent.getParcelableExtra(
                BluetoothDevice.EXTRA_DEVICE,
                BluetoothDevice::class.java
            )

        } else {

            intent.getParcelableExtra(
                BluetoothDevice.EXTRA_DEVICE
            )
        }
    }


    // ============================================================
    // PERMISSIONS
    // ============================================================

    private fun hasConnectPermission(): Boolean {

        if (
            Build.VERSION.SDK_INT <
            Build.VERSION_CODES.S
        ) {
            return true
        }


        return ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.BLUETOOTH_CONNECT
        ) == PackageManager.PERMISSION_GRANTED
    }


    private fun hasScanPermission(): Boolean {

        if (
            Build.VERSION.SDK_INT <
            Build.VERSION_CODES.S
        ) {
            return true
        }


        return ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.BLUETOOTH_SCAN
        ) == PackageManager.PERMISSION_GRANTED
    }


    // ============================================================
    // CLEANUP
    // ============================================================

    fun destroy() {

        stopDiscovery()


        if (receiverRegistered) {

            try {

                context.unregisterReceiver(
                    receiver
                )

            } catch (_: Exception) {
            }


            receiverRegistered = false
        }
    }
}