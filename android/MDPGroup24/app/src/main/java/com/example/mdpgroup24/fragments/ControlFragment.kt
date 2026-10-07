package com.example.mdpgroup24.fragments

import android.content.res.ColorStateList
import android.os.Bundle
import android.view.View
import android.widget.Button
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import com.example.mdpgroup24.R
import com.example.mdpgroup24.bluetooth.BluetoothSession

class ControlFragment :
    Fragment(R.layout.fragment_control) {

    override fun onViewCreated(
        view: View,
        savedInstanceState: Bundle?
    ) {

        super.onViewCreated(
            view,
            savedInstanceState
        )


        val btnForward =
            view.findViewById<Button>(
                R.id.btnForward
            )

        val btnBackward =
            view.findViewById<Button>(
                R.id.btnBackward
            )

        val btnLeft =
            view.findViewById<Button>(
                R.id.btnLeft
            )

        val btnRight =
            view.findViewById<Button>(
                R.id.btnRight
            )


        // =====================================================
        // BUTTON APPEARANCE
        // =====================================================

        applyControlButtonStyle(
            btnForward
        )

        applyControlButtonStyle(
            btnBackward
        )

        applyControlButtonStyle(
            btnLeft
        )

        applyControlButtonStyle(
            btnRight
        )


        // =====================================================
        // CONTROL ACTIONS
        // =====================================================

        btnForward.setOnClickListener {

            performControlAction(
                "F"
            ) { home ->

                home.moveRobotForward()
            }
        }


        btnBackward.setOnClickListener {

            performControlAction(
                "B"
            ) { home ->

                home.moveRobotBackward()
            }
        }


        btnLeft.setOnClickListener {

            performControlAction(
                "L"
            ) { home ->

                home.rotateRobotLeft()
            }
        }


        btnRight.setOnClickListener {

            performControlAction(
                "R"
            ) { home ->

                home.rotateRobotRight()
            }
        }
    }


    // =====================================================
    // BUTTON STYLE
    // =====================================================

    private fun applyControlButtonStyle(
        button: Button
    ) {

        val inactiveBackground =
            ContextCompat.getColor(
                requireContext(),
                R.color.control_inactive
            )


        val pressedBackground =
            ContextCompat.getColor(
                requireContext(),
                R.color.odyssey_bronze_light
            )


        val inactiveText =
            ContextCompat.getColor(
                requireContext(),
                R.color.odyssey_text_muted
            )


        val pressedText =
            ContextCompat.getColor(
                requireContext(),
                R.color.white
            )


        button.backgroundTintList =
            ColorStateList(
                arrayOf(
                    intArrayOf(
                        android.R.attr.state_pressed
                    ),
                    intArrayOf()
                ),
                intArrayOf(
                    pressedBackground,
                    inactiveBackground
                )
            )


        button.setTextColor(
            ColorStateList(
                arrayOf(
                    intArrayOf(
                        android.R.attr.state_pressed
                    ),
                    intArrayOf()
                ),
                intArrayOf(
                    pressedText,
                    inactiveText
                )
            )
        )
    }


    // =====================================================
    // CONTROL LOGIC
    // =====================================================

    private fun performControlAction(
        bluetoothCommand: String,
        action: (HomeFragment) -> Boolean
    ) {

        val home =
            parentFragment as? HomeFragment
                ?: return

        if (BluetoothSession.isConnected) {
            BluetoothSession.sendManual(bluetoothCommand)
            return
        }


        val success =
            action(
                home
            )


        if (!success) {

            return
        }


        // Offline map controls remain available for the simulator/checklist.
    }
}
