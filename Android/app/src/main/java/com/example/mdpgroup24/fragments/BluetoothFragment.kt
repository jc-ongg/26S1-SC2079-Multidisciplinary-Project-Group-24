package com.example.mdpgroup24.fragments

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import android.content.res.ColorStateList
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import com.example.mdpgroup24.R
import com.example.mdpgroup24.bluetooth.BluetoothController
import com.example.mdpgroup24.bluetooth.BluetoothSession
import com.example.mdpgroup24.bluetooth.BluetoothConnectionService

class BluetoothFragment :
    Fragment(R.layout.fragment_bluetooth),
    BluetoothController.Listener,
    BluetoothSession.Listener {

    private lateinit var bluetoothController: BluetoothController

    private lateinit var bluetoothStatus: TextView
    private lateinit var connectionStatus: TextView
    private lateinit var connectedDevice: TextView

    private lateinit var pairedDevicesContainer: LinearLayout
    private lateinit var nearbyDevicesContainer: LinearLayout

    private lateinit var refreshButton: Button
    private lateinit var disconnectButton: Button
    private var nearbyDeviceFound = false

    private val permissionLauncher =
        registerForActivityResult(
            ActivityResultContracts.RequestMultiplePermissions()
        ) {
            refreshBluetoothInfo()
        }

    override fun onViewCreated(
        view: View,
        savedInstanceState: Bundle?
    ) {
        super.onViewCreated(view, savedInstanceState)

        bluetoothStatus = view.findViewById(R.id.bluetoothStatus)
        connectionStatus = view.findViewById(R.id.connectionStatus)
        connectedDevice = view.findViewById(R.id.connectedDevice)

        pairedDevicesContainer =
            view.findViewById(R.id.pairedDevicesContainer)

        nearbyDevicesContainer =
            view.findViewById(R.id.nearbyDevicesContainer)

        refreshButton =
            view.findViewById(R.id.btnRefresh)

        disconnectButton =
            view.findViewById(R.id.btnDisconnect)

        bluetoothController =
            BluetoothController(
                requireContext(),
                this
            )

        BluetoothSession.initialize(requireContext())
        BluetoothSession.addListener(this)

        bluetoothController.registerReceiver()

        refreshButton.setOnClickListener {
            refreshBluetoothInfo()
        }

        disconnectButton.setOnClickListener {

            if (!BluetoothSession.isConnected) {
                return@setOnClickListener
            }

            bluetoothController.stopDiscovery()

            BluetoothSession.disconnect()
        }

        checkBluetoothPermissions()
    }

    override fun onResume() {
        super.onResume()

        if (::bluetoothController.isInitialized) {
            updateBluetoothStatus()
        }
    }

    // ============================================================
    // PERMISSIONS
    // ============================================================

    private fun checkBluetoothPermissions() {

        val permissions =
            mutableListOf<String>()

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {

            if (!hasConnectPermission()) {
                permissions.add(
                    Manifest.permission.BLUETOOTH_CONNECT
                )
            }

            if (!hasScanPermission()) {
                permissions.add(
                    Manifest.permission.BLUETOOTH_SCAN
                )
            }

        } else if (
            Build.VERSION.SDK_INT >= Build.VERSION_CODES.M &&
            !hasScanPermission()
        ) {

            permissions.add(
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }

        if (permissions.isEmpty()) {
            refreshBluetoothInfo()
        } else {
            permissionLauncher.launch(
                permissions.toTypedArray()
            )
        }
    }

    private fun hasConnectPermission(): Boolean {

        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S) {
            return true
        }

        return ContextCompat.checkSelfPermission(
            requireContext(),
            Manifest.permission.BLUETOOTH_CONNECT
        ) == PackageManager.PERMISSION_GRANTED
    }

    private fun hasScanPermission(): Boolean {

        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {

            ContextCompat.checkSelfPermission(
                requireContext(),
                Manifest.permission.BLUETOOTH_SCAN
            ) == PackageManager.PERMISSION_GRANTED

        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {

            ContextCompat.checkSelfPermission(
                requireContext(),
                Manifest.permission.ACCESS_FINE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED

        } else {
            true
        }
    }

    // ============================================================
    // REFRESH
    // ============================================================

    private fun refreshBluetoothInfo() {

        updateBluetoothStatus()

        if (!hasConnectPermission()) {
            showPairedMessage("Bluetooth permission required")
            showNearbyMessage("Bluetooth permission required")
            return
        }

        if (!bluetoothController.isAvailable()) {
            showPairedMessage("Bluetooth unavailable")
            showNearbyMessage("Bluetooth unavailable")
            return
        }

        if (!bluetoothController.isEnabled()) {
            showPairedMessage("Bluetooth is off")
            showNearbyMessage("Bluetooth is off")
            return
        }

        BluetoothSession.ensureListening()

        // BluetoothSession.startListening()

        loadPairedDevices()

        if (!hasScanPermission()) {
            showNearbyMessage("Bluetooth scan permission required")
            return
        }

        nearbyDeviceFound = false
        showNearbyMessage("Scanning...")

        if (!bluetoothController.startDiscovery()) {
            showNearbyMessage("Unable to start scan")
        }
    }

    private fun updateBluetoothStatus() {

        when {

            !hasConnectPermission() -> {
                bluetoothStatus.text = "● Permission Required"
                bluetoothStatus.setTextColor(
                    color(R.color.status_orange)
                )
            }

            !bluetoothController.isAvailable() -> {
                bluetoothStatus.text = "● Unavailable"
                bluetoothStatus.setTextColor(
                    color(R.color.status_red)
                )
            }

            bluetoothController.isEnabled() -> {
                bluetoothStatus.text = "● ON"
                bluetoothStatus.setTextColor(
                    color(R.color.status_green)
                )
            }

            else -> {
                bluetoothStatus.text = "● OFF"
                bluetoothStatus.setTextColor(
                    color(R.color.status_red)
                )
            }
        }
    }

    // ============================================================
    // PAIRED DEVICES
    // ============================================================

    @SuppressLint("MissingPermission")
    private fun loadPairedDevices() {

        pairedDevicesContainer.removeAllViews()

        val devices =
            bluetoothController.getPairedDevices()

        if (devices.isEmpty()) {
            showPairedMessage("No paired devices")
            return
        }

        devices.forEach { device ->

            val row =
                createDeviceRow(device)

            row.setOnClickListener {

                bluetoothController
                    .stopDiscovery()


                BluetoothSession.connect(
                    device
                )
            }

            pairedDevicesContainer.addView(row)
        }
    }

    // ============================================================
    // DISCOVERY / PAIRING CALLBACKS
    // ============================================================

    @SuppressLint("MissingPermission")
    override fun onDeviceFound(
        device: BluetoothDevice
    ) {

        if (!nearbyDeviceFound) {
            nearbyDevicesContainer.removeAllViews()
            nearbyDeviceFound = true
        }

        val row =
            createDeviceRow(device)

        row.setOnClickListener {

            val pairingStarted =
                bluetoothController.pairDevice(device)

            if (!pairingStarted) {
                showNearbyMessage(
                    "Unable to start pairing"
                )
            }
        }

        nearbyDevicesContainer.addView(row)
    }

    override fun onDiscoveryFinished(
        foundAny: Boolean
    ) {

        if (!foundAny) {
            showNearbyMessage(
                "No nearby devices found"
            )
        }
    }

    override fun onDevicePaired(
        device: BluetoothDevice
    ) {

        refreshBluetoothInfo()
    }

    // ============================================================
    // CONNECTION CALLBACKS
    // ============================================================

    @SuppressLint("MissingPermission")
    override fun onConnectionStateChanged(
        state: BluetoothConnectionService.ConnectionState,
        device: BluetoothDevice?
    ) {

        if (!isAdded || view == null) {
            return
        }

        val deviceName =
            device?.name ?: "--"

        when (state) {

            BluetoothConnectionService.ConnectionState.CONNECTING -> {

                connectionStatus.text = "Connecting..."
                connectionStatus.setTextColor(
                    color(R.color.status_orange)
                )

                connectedDevice.text =
                    deviceName

                setDisconnectButtonEnabled(false)

            }

            BluetoothConnectionService.ConnectionState.CONNECTED -> {

                bluetoothController
                    .stopDiscovery()

                connectionStatus.text = "Connected"
                connectionStatus.setTextColor(
                    color(R.color.status_green)
                )

                connectedDevice.text =
                    deviceName

                setDisconnectButtonEnabled(true)

            }

            BluetoothConnectionService.ConnectionState.FAILED -> {

                connectionStatus.text = "Connection Failed"
                connectionStatus.setTextColor(
                    color(R.color.status_red)
                )

                connectedDevice.text = "--"

                setDisconnectButtonEnabled(false)

            }

            BluetoothConnectionService.ConnectionState.DISCONNECTED -> {

                connectionStatus.text = "Disconnected"
                connectionStatus.setTextColor(
                    color(R.color.odyssey_text_muted)
                )

                connectedDevice.text = "--"

                setDisconnectButtonEnabled(false)

            }
        }
    }

    override fun onMessageReceived(
        message: String
    ) {

        Log.d(
            "BluetoothFragment",
            "Received: $message"
        )

        // Later:
        // forward message to chat / map / robot state parser
    }

    // ============================================================
    // UI HELPERS
    // ============================================================

    private fun setDisconnectButtonEnabled(
        enabled: Boolean
    ) {

        disconnectButton.isEnabled =
            enabled


        if (enabled) {

            disconnectButton.backgroundTintList =
                ColorStateList.valueOf(
                    color(R.color.status_red)
                )

            disconnectButton.setTextColor(
                color(R.color.white)
            )

        } else {

            disconnectButton.backgroundTintList =
                ColorStateList.valueOf(
                    color(R.color.control_inactive)
                )

            disconnectButton.setTextColor(
                color(R.color.odyssey_text_muted)
            )
        }
    }


    @SuppressLint("MissingPermission")
    private fun createDeviceRow(
        device: BluetoothDevice
    ): TextView {

        val name =
            device.name ?: "Unknown Device"

        return TextView(requireContext()).apply {

            text = "$name\n${device.address}"
            textSize = 15f

            setTextColor(
                color(R.color.odyssey_text)
            )

            setPadding(
                dpToPx(12),
                dpToPx(10),
                dpToPx(12),
                dpToPx(10)
            )

            setBackgroundColor(
                color(R.color.card_background)
            )

            isClickable = true
            isFocusable = true

            layoutParams =
                LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT
                ).apply {
                    bottomMargin = dpToPx(8)
                }
        }
    }

    private fun showPairedMessage(
        message: String
    ) {

        pairedDevicesContainer.removeAllViews()
        pairedDevicesContainer.addView(
            createMessageView(message)
        )
    }

    private fun showNearbyMessage(
        message: String
    ) {

        nearbyDevicesContainer.removeAllViews()
        nearbyDevicesContainer.addView(
            createMessageView(message)
        )
    }

    private fun createMessageView(
        message: String
    ): TextView {

        return TextView(requireContext()).apply {

            text = message
            textSize = 14f

            setTextColor(
                color(R.color.odyssey_text_muted)
            )

            setPadding(
                dpToPx(8),
                dpToPx(12),
                dpToPx(8),
                dpToPx(12)
            )
        }
    }

    private fun color(resource: Int): Int {
        return ContextCompat.getColor(
            requireContext(),
            resource
        )
    }

    private fun dpToPx(dp: Int): Int {
        return (
                dp * resources.displayMetrics.density
                ).toInt()
    }

    // ============================================================
    // CLEANUP
    // ============================================================

    override fun onDestroyView() {

        BluetoothSession.removeListener(this)

        if (::bluetoothController.isInitialized) {
            bluetoothController.destroy()
        }

        super.onDestroyView()
    }
}