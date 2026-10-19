package com.example.mdpgroup24.fragments

import android.content.res.ColorStateList
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.view.View
import android.widget.Button
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import com.example.mdpgroup24.R
import com.example.mdpgroup24.bluetooth.BluetoothSession
import com.example.mdpgroup24.bluetooth.BluetoothConnectionService
import com.example.mdpgroup24.bluetooth.Task1Preparation
import android.bluetooth.BluetoothDevice
import android.widget.Toast

class ChallengeFragment :
    Fragment(R.layout.fragment_challenge), BluetoothSession.Listener {

    private lateinit var timerText: TextView
    private lateinit var statusText: TextView

    private lateinit var task1Button: Button
    private lateinit var task2Button: Button

    private lateinit var startButton: Button
    private lateinit var stopButton: Button


    private val timerHandler =
        Handler(
            Looper.getMainLooper()
        )


    private var startTime =
        0L

    private var elapsedTime =
        0L

    private var timerRunning =
        false


    // Task 1 is selected by default.
    private var selectedTask =
        1


    private enum class ChallengeState {
        READY,
        RUNNING,
        FINISHED
    }


    private var challengeState =
        ChallengeState.READY


    private val timerRunnable =
        object : Runnable {

            override fun run() {

                if (!timerRunning) {
                    return
                }


                elapsedTime =
                    SystemClock.elapsedRealtime() -
                            startTime


                timerText.text =
                    formatTime(
                        elapsedTime
                    )


                timerHandler.postDelayed(
                    this,
                    25L
                )
            }
        }


    override fun onViewCreated(
        view: View,
        savedInstanceState: Bundle?
    ) {

        super.onViewCreated(
            view,
            savedInstanceState
        )


        timerText =
            view.findViewById(
                R.id.challengeTimer
            )


        statusText =
            view.findViewById(
                R.id.challengeStatus
            )


        task1Button =
            view.findViewById(
                R.id.btnTask1
            )


        task2Button =
            view.findViewById(
                R.id.btnTask2
            )


        startButton =
            view.findViewById(
                R.id.btnChallengeStart
            )


        stopButton =
            view.findViewById(
                R.id.btnChallengeStop
            )


        setupTaskSelection()

        BluetoothSession.task1StartedAt?.let { started ->
            selectedTask = 1
            startTime = started
            val finished = BluetoothSession.task1FinishedAt
            timerRunning = finished == null
            elapsedTime = (finished ?: SystemClock.elapsedRealtime()) - started
            challengeState = if (timerRunning) ChallengeState.RUNNING else ChallengeState.FINISHED
        }
        timerText.text = formatTime(elapsedTime)
        if (timerRunning) timerHandler.post(timerRunnable)

        BluetoothSession.addListener(this)

        updateChallengeUi()


        // =====================================================
        // START
        // =====================================================

        startButton.setOnClickListener {

            if (
                challengeState !=
                ChallengeState.READY
            ) {
                return@setOnClickListener
            }


            startChallenge()
        }


        // =====================================================
        // STOP / RESET
        // =====================================================

        stopButton.setOnClickListener {

            when (challengeState) {

                ChallengeState.RUNNING -> {

                    stopChallenge()
                }


                ChallengeState.FINISHED -> {

                    resetChallenge()
                }


                ChallengeState.READY -> {

                    // Disabled in this state.
                }
            }
        }
    }


    // =====================================================
    // TASK SELECTION
    // =====================================================

    private fun setupTaskSelection() {

        task1Button.setOnClickListener {

            if (
                challengeState !=
                ChallengeState.READY
            ) {
                return@setOnClickListener
            }


            selectedTask =
                1


            updateChallengeUi()
        }


        task2Button.setOnClickListener {

            if (
                challengeState !=
                ChallengeState.READY
            ) {
                return@setOnClickListener
            }


            selectedTask =
                2


            updateChallengeUi()
        }


        updateTaskSelectionUi()
    }


    private fun updateTaskSelectionUi() {

        if (selectedTask == 1) {

            setButtonStyle(
                task1Button,
                R.color.odyssey_navy,
                R.color.white
            )


            setButtonStyle(
                task2Button,
                R.color.control_inactive,
                R.color.odyssey_text_muted
            )

        } else {

            setButtonStyle(
                task1Button,
                R.color.control_inactive,
                R.color.odyssey_text_muted
            )


            setButtonStyle(
                task2Button,
                R.color.odyssey_navy,
                R.color.white
            )
        }
    }


    // =====================================================
    // START
    // =====================================================

    private fun startChallenge() {

        val sent = if (selectedTask == 1) BluetoothSession.startTask1()
                   else sendProtocolMessage("BEGIN,$selectedTask")
        if (!sent) {
            Toast.makeText(requireContext(), "Connect Bluetooth and send the map to prepare a route first.",
                Toast.LENGTH_SHORT).show()
            updateChallengeUi()
            return
        }

        elapsedTime =
            0L


        startTime =
            if (selectedTask == 1) BluetoothSession.task1StartedAt ?: SystemClock.elapsedRealtime()
            else SystemClock.elapsedRealtime()


        timerRunning =
            true


        challengeState =
            ChallengeState.RUNNING


        timerHandler.post(
            timerRunnable
        )


        updateChallengeUi()
    }


    // =====================================================
    // STOP
    // =====================================================

    private fun stopChallenge(sendResult: Boolean = true) {

        if (!timerRunning) {
            return
        }


        elapsedTime =
            (if (selectedTask == 1) BluetoothSession.task1FinishedAt ?: SystemClock.elapsedRealtime()
             else SystemClock.elapsedRealtime()) -
                    startTime


        timerRunning =
            false


        timerHandler.removeCallbacks(
            timerRunnable
        )


        timerText.text =
            formatTime(
                elapsedTime
            )


        challengeState =
            ChallengeState.FINISHED


        if (sendResult) {
            sendProtocolMessage("RESULT,$selectedTask,${formatTime(elapsedTime)}")
        }
        if (selectedTask == 1) {
            BluetoothSession.finishTask1Timer()
            BluetoothSession.invalidatePreparation()
        }


        updateChallengeUi()
    }


    // =====================================================
    // RESET
    // =====================================================

    private fun resetChallenge() {

        if (selectedTask == 1) BluetoothSession.resetTask1Timer()

        timerRunning =
            false


        timerHandler.removeCallbacks(
            timerRunnable
        )


        elapsedTime =
            0L


        timerText.text =
            "00:00.00"


        challengeState =
            ChallengeState.READY


        updateChallengeUi()
    }


    // =====================================================
    // UI STATE
    // =====================================================

    private fun updateChallengeUi() {

        when (challengeState) {

            // -------------------------------------------------
            // READY
            // -------------------------------------------------

            ChallengeState.READY -> {

                val canStart = BluetoothSession.isConnected &&
                    (selectedTask != 1 || BluetoothSession.preparation.canStart)

                statusText.text = when {
                    !BluetoothSession.isConnected -> "Connect Bluetooth"
                    selectedTask != 1 -> "Ready"
                    BluetoothSession.preparation.state == Task1Preparation.State.PLANNING -> "Preparing route…"
                    BluetoothSession.preparation.state == Task1Preparation.State.ERROR ->
                        BluetoothSession.preparation.error ?: "Route failed. Send map again."
                    canStart -> "Route ready"
                    else -> "Send map to prepare route"
                }


                statusText.setTextColor(
                    color(
                        R.color.odyssey_text_muted
                    )
                )


                task1Button.isEnabled =
                    true

                task2Button.isEnabled =
                    true


                updateTaskSelectionUi()


                startButton.isEnabled = canStart

                startButton.text =
                    "Start"

                setButtonStyle(
                    startButton,
                    if (canStart) R.color.control_active else R.color.control_inactive,
                    if (canStart) R.color.white else R.color.odyssey_text_muted
                )


                stopButton.isEnabled =
                    false

                stopButton.text =
                    "Stop"

                setButtonStyle(
                    stopButton,
                    R.color.control_inactive,
                    R.color.odyssey_text_muted
                )
            }


            // -------------------------------------------------
            // RUNNING
            // -------------------------------------------------

            ChallengeState.RUNNING -> {

                statusText.text =
                    "Running"


                statusText.setTextColor(
                    color(
                        R.color.status_green
                    )
                )


                task1Button.isEnabled =
                    false

                task2Button.isEnabled =
                    false


                startButton.isEnabled =
                    false

                setButtonStyle(
                    startButton,
                    R.color.control_inactive,
                    R.color.odyssey_text_muted
                )


                stopButton.isEnabled =
                    true

                stopButton.text =
                    "Stop"

                setButtonStyle(
                    stopButton,
                    R.color.status_red,
                    R.color.white
                )
            }


            // -------------------------------------------------
            // FINISHED
            // -------------------------------------------------

            ChallengeState.FINISHED -> {

                statusText.text =
                    "Finished"


                statusText.setTextColor(
                    color(
                        R.color.status_red
                    )
                )


                task1Button.isEnabled =
                    false

                task2Button.isEnabled =
                    false


                startButton.isEnabled =
                    false

                setButtonStyle(
                    startButton,
                    R.color.control_inactive,
                    R.color.odyssey_text_muted
                )


                stopButton.isEnabled =
                    true

                stopButton.text =
                    "Reset"

                setButtonStyle(
                    stopButton,
                    R.color.odyssey_bronze_light,
                    R.color.white
                )
            }
        }
    }


    // =====================================================
    // TIME FORMAT
    // =====================================================

    private fun formatTime(
        milliseconds: Long
    ): String {

        val totalCentiseconds =
            milliseconds / 10


        val minutes =
            totalCentiseconds /
                    6000


        val seconds =
            (
                    totalCentiseconds /
                            100
                    ) % 60


        val centiseconds =
            totalCentiseconds %
                    100


        return String.format(
            "%02d:%02d.%02d",
            minutes,
            seconds,
            centiseconds
        )
    }


    // =====================================================
    // BLUETOOTH
    // =====================================================

    private fun sendProtocolMessage(
        message: String
    ): Boolean {

        if (!BluetoothSession.isConnected) {
            return false
        }


        return BluetoothSession.send(
            "//$message//"
        )
    }


    // =====================================================
    // UI HELPERS
    // =====================================================

    private fun setButtonStyle(
        button: Button,
        backgroundColor: Int,
        textColor: Int
    ) {

        button.backgroundTintList =
            ColorStateList.valueOf(
                color(
                    backgroundColor
                )
            )


        button.setTextColor(
            color(
                textColor
            )
        )
    }


    private fun color(
        resource: Int
    ): Int {

        return ContextCompat.getColor(
            requireContext(),
            resource
        )
    }


    // =====================================================
    // CLEANUP
    // =====================================================

    override fun onDestroyView() {

        BluetoothSession.removeListener(this)

        timerHandler.removeCallbacks(
            timerRunnable
        )


        super.onDestroyView()
    }

    override fun onMessageReceived(message: String) {}

    override fun onConnectionStateChanged(
        state: BluetoothConnectionService.ConnectionState,
        device: BluetoothDevice?
    ) {
        if (view != null && ::startButton.isInitialized) updateChallengeUi()
    }

    override fun onPreparationChanged() {
        if (view != null && ::startButton.isInitialized) updateChallengeUi()
    }

    override fun onTaskStatus(status: String) {
        if (selectedTask == 1 && timerRunning && status.equals("finished", true)) {
            stopChallenge(sendResult = false)
        }
    }
}
