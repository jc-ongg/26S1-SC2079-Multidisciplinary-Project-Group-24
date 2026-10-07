package com.example.mdpgroup24.fragments

import android.content.res.ColorStateList
import android.graphics.Typeface
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.LinearLayout
import android.widget.NumberPicker
import android.widget.TextView
import android.widget.Toast
import androidx.core.content.ContextCompat
import androidx.core.os.bundleOf
import androidx.fragment.app.Fragment
import com.example.mdpgroup24.R
import com.example.mdpgroup24.views.ArenaGridView
import androidx.appcompat.widget.SwitchCompat


class ConfigurationFragment :
    Fragment(R.layout.fragment_configuration) {


    // =====================================================
    // TYPES
    // =====================================================

    private enum class ConfigurationTab {
        PLACE_ROBOT,
        PLACE_OBSTACLE,
        INPUT_OBSTACLE
    }


    private enum class Direction {
        NORTH,
        EAST,
        SOUTH,
        WEST
    }


    // =====================================================
    // STATE
    // =====================================================

    private var selectedTab =
        ConfigurationTab.PLACE_ROBOT


    private var obstaclePlacementActive =
        false

    private var robotPlacementActive =
        false


    private var pendingManualX: Int? =
        null

    private var pendingManualY: Int? =
        null


    private var pendingRobotX: Int? =
        null

    private var pendingRobotY: Int? =
        null


    private var selectedInputDirection =
        Direction.NORTH


    // =====================================================
    // TAB UI
    // =====================================================

    private lateinit var placeRobotTabButton:
            Button

    private lateinit var placeObstacleTabButton:
            Button

    private lateinit var inputObstacleTabButton:
            Button


    private lateinit var robotPanel:
            LinearLayout

    private lateinit var obstacleManualPanel:
            LinearLayout

    private lateinit var obstacleInputPanel:
            LinearLayout


    private lateinit var undoMapButton:
            Button

    private lateinit var saveMapButton:
            Button

    private lateinit var useMapButton:
            Button

    private lateinit var resetMapButton:
            Button

    private lateinit var sendMapButton:
            Button


    // =====================================================
    // ROBOT UI
    // =====================================================

    private lateinit var setRobotButton:
            SwitchCompat

    private lateinit var robotOrientationPanel:
            LinearLayout

    private lateinit var robotCentreLabel:
            TextView

    private lateinit var robotOrientationLabel:
            TextView

    private lateinit var robotSelectedCellText:
            TextView


    private lateinit var robotNorthButton:
            Button

    private lateinit var robotSouthButton:
            Button

    private lateinit var robotEastButton:
            Button

    private lateinit var robotWestButton:
            Button


    // =====================================================
    // MANUAL OBSTACLE UI
    // =====================================================

    private lateinit var activateObstaclePlacementButton:
            SwitchCompat

    private lateinit var manualOrientationPanel:
            LinearLayout

    private lateinit var manualCellLabel:
            TextView

    private lateinit var manualOrientationLabel:
            TextView

    private lateinit var manualSelectedCellText:
            TextView


    private lateinit var manualNorthButton:
            Button

    private lateinit var manualSouthButton:
            Button

    private lateinit var manualEastButton:
            Button

    private lateinit var manualWestButton:
            Button


    // =====================================================
    // INPUT OBSTACLE UI
    // =====================================================

    private lateinit var xPicker:
            NumberPicker

    private lateinit var yPicker:
            NumberPicker


    private lateinit var inputNorthButton:
            Button

    private lateinit var inputSouthButton:
            Button

    private lateinit var inputEastButton:
            Button

    private lateinit var inputWestButton:
            Button


    private lateinit var placeObstacleButton:
            Button


    // =====================================================
    // CREATE
    // =====================================================

    override fun onViewCreated(
        view: View,
        savedInstanceState: Bundle?
    ) {

        super.onViewCreated(
            view,
            savedInstanceState
        )


        bindViews(view)


        setupTabs()

        setupRobotPlacement()
        setupRobotOrientation()

        setupManualObstaclePlacement()
        setupManualOrientation()

        setupCoordinatePickers()
        setupInputOrientation()
        setupInputPlacement()

        setupMapActions()

        setupFragmentResults()


        // Default configuration tab.
        selectTab(
            ConfigurationTab.PLACE_ROBOT
        )


        // Default direction for coordinate-input obstacle.
        selectInputDirection(
            Direction.NORTH
        )
    }


    // =====================================================
    // BIND VIEWS
    // =====================================================

    private fun bindViews(
        view: View
    ) {

        // -------------------------------------------------
        // TABS
        // -------------------------------------------------

        placeRobotTabButton =
            view.findViewById(
                R.id.btnTabPlaceRobot
            )

        placeObstacleTabButton =
            view.findViewById(
                R.id.btnTabPlaceObstacle
            )

        inputObstacleTabButton =
            view.findViewById(
                R.id.btnTabInputObstacle
            )


        // -------------------------------------------------
        // PANELS
        // -------------------------------------------------

        robotPanel =
            view.findViewById(
                R.id.robotPanel
            )

        obstacleManualPanel =
            view.findViewById(
                R.id.obstacleManualPanel
            )

        obstacleInputPanel =
            view.findViewById(
                R.id.obstacleInputPanel
            )


        // -------------------------------------------------
        // ROBOT
        // -------------------------------------------------

        setRobotButton =
            view.findViewById(
                R.id.btnSetRobot
            )

        robotOrientationPanel =
            view.findViewById(
                R.id.robotOrientationPanel
            )

        robotCentreLabel =
            view.findViewById(
                R.id.robotCentreLabel
            )

        robotOrientationLabel =
            view.findViewById(
                R.id.robotOrientationLabel
            )

        robotSelectedCellText =
            view.findViewById(
                R.id.robotSelectedCellText
            )


        robotNorthButton =
            view.findViewById(
                R.id.btnRobotNorth
            )

        robotSouthButton =
            view.findViewById(
                R.id.btnRobotSouth
            )

        robotEastButton =
            view.findViewById(
                R.id.btnRobotEast
            )

        robotWestButton =
            view.findViewById(
                R.id.btnRobotWest
            )


        // -------------------------------------------------
        // MANUAL OBSTACLE
        // -------------------------------------------------

        activateObstaclePlacementButton =
            view.findViewById(
                R.id.btnActivateObstaclePlacement
            )

        manualOrientationPanel =
            view.findViewById(
                R.id.manualOrientationPanel
            )

        manualCellLabel =
            view.findViewById(
                R.id.manualCellLabel
            )

        manualOrientationLabel =
            view.findViewById(
                R.id.manualOrientationLabel
            )

        manualSelectedCellText =
            view.findViewById(
                R.id.manualSelectedCellText
            )


        manualNorthButton =
            view.findViewById(
                R.id.btnManualNorth
            )

        manualSouthButton =
            view.findViewById(
                R.id.btnManualSouth
            )

        manualEastButton =
            view.findViewById(
                R.id.btnManualEast
            )

        manualWestButton =
            view.findViewById(
                R.id.btnManualWest
            )


        // -------------------------------------------------
        // INPUT OBSTACLE
        // -------------------------------------------------

        xPicker =
            view.findViewById(
                R.id.obstacleXPicker
            )

        yPicker =
            view.findViewById(
                R.id.obstacleYPicker
            )


        inputNorthButton =
            view.findViewById(
                R.id.btnObstacleNorth
            )

        inputSouthButton =
            view.findViewById(
                R.id.btnObstacleSouth
            )

        inputEastButton =
            view.findViewById(
                R.id.btnObstacleEast
            )

        inputWestButton =
            view.findViewById(
                R.id.btnObstacleWest
            )


        placeObstacleButton =
            view.findViewById(
                R.id.btnPlaceObstacle
            )


        // -------------------------------------------------
        // MAP ACTIONS
        // -------------------------------------------------

        undoMapButton =
            view.findViewById(
                R.id.btnUndoMap
            )

        saveMapButton =
            view.findViewById(
                R.id.btnSaveMap
            )

        useMapButton =
            view.findViewById(
                R.id.btnUseMap
            )

        resetMapButton =
            view.findViewById(
                R.id.btnResetMap
            )

        sendMapButton =
            view.findViewById(
                R.id.btnSendMap
            )
    }


    // =====================================================
    // CONFIGURATION TABS
    // =====================================================

    private fun setupTabs() {

        placeRobotTabButton
            .setOnClickListener {

                selectTab(
                    ConfigurationTab.PLACE_ROBOT
                )
            }


        placeObstacleTabButton
            .setOnClickListener {

                selectTab(
                    ConfigurationTab.PLACE_OBSTACLE
                )
            }


        inputObstacleTabButton
            .setOnClickListener {

                selectTab(
                    ConfigurationTab.INPUT_OBSTACLE
                )
            }
    }


    private fun selectTab(
        tab: ConfigurationTab
    ) {

        // Stop robot placement when leaving its tab.
        if (
            tab != ConfigurationTab.PLACE_ROBOT &&
            robotPlacementActive
        ) {

            deactivateRobotPlacement()
        }


        // Stop manual obstacle placement when leaving its tab.
        if (
            tab != ConfigurationTab.PLACE_OBSTACLE &&
            obstaclePlacementActive
        ) {

            deactivateObstaclePlacement()
        }


        selectedTab =
            tab


        // -------------------------------------------------
        // PANEL VISIBILITY
        // -------------------------------------------------

        robotPanel.visibility =
            if (
                tab ==
                ConfigurationTab.PLACE_ROBOT
            ) {
                View.VISIBLE
            } else {
                View.GONE
            }


        obstacleManualPanel.visibility =
            if (
                tab ==
                ConfigurationTab.PLACE_OBSTACLE
            ) {
                View.VISIBLE
            } else {
                View.GONE
            }


        obstacleInputPanel.visibility =
            if (
                tab ==
                ConfigurationTab.INPUT_OBSTACLE
            ) {
                View.VISIBLE
            } else {
                View.GONE
            }


        // -------------------------------------------------
        // TAB STYLE
        // -------------------------------------------------

        setUnselectedTab(
            placeRobotTabButton
        )

        setUnselectedTab(
            placeObstacleTabButton
        )

        setUnselectedTab(
            inputObstacleTabButton
        )


        when (tab) {

            ConfigurationTab.PLACE_ROBOT -> {

                setSelectedTab(
                    placeRobotTabButton
                )
            }


            ConfigurationTab.PLACE_OBSTACLE -> {

                setSelectedTab(
                    placeObstacleTabButton
                )
            }


            ConfigurationTab.INPUT_OBSTACLE -> {

                setSelectedTab(
                    inputObstacleTabButton
                )
            }
        }
    }


    // =====================================================
    // ROBOT PLACEMENT
    // =====================================================

    private fun setupRobotPlacement() {

        setupPlacementSwitch(
            setRobotButton
        )


        setRobotButton
            .setOnCheckedChangeListener { _, isChecked ->

                if (isChecked) {

                    if (obstaclePlacementActive) {

                        deactivateObstaclePlacement()
                    }


                    if (!robotPlacementActive) {

                        activateRobotPlacement()
                    }

                } else {

                    if (robotPlacementActive) {

                        deactivateRobotPlacement()
                    }
                }
            }
    }

    private fun activateRobotPlacement() {

        robotPlacementActive =
            true


        clearPendingRobotSelection()


        // Show both placement steps immediately.
        robotOrientationPanel.visibility =
            View.VISIBLE

        setRobotButton.isChecked =
            true


        updatePlacementSwitchAppearance(
            setRobotButton,
            true
        )



        (parentFragment as? HomeFragment)
            ?.setArenaSelectionMode(
                ArenaGridView.SelectionMode.ROBOT
            )
    }

    private fun deactivateRobotPlacement(
        notifyHome: Boolean = true
    ) {

        robotPlacementActive =
            false


        setRobotButton.isChecked =
            false


        updatePlacementSwitchAppearance(
            setRobotButton,
            false
        )


        clearPendingRobotSelection()


        robotOrientationPanel.visibility =
            View.GONE


        if (notifyHome) {

            (parentFragment as? HomeFragment)
                ?.setArenaSelectionMode(
                    ArenaGridView.SelectionMode.NONE
                )
        }
    }


    fun onRobotGridSelected(
        x: Int,
        y: Int
    ) {

        if (
            !robotPlacementActive ||
            selectedTab !=
            ConfigurationTab.PLACE_ROBOT
        ) {
            return
        }


        pendingRobotX =
            x

        pendingRobotY =
            y


        robotSelectedCellText.text =
            "X: $x   Y: $y"


        updateRobotPlacementStep(
            centreSelected = true
        )


        setRobotDirectionButtonsEnabled(
            true
        )
    }


    fun onInvalidRobotGridSelection() {

        if (!robotPlacementActive) {
            return
        }


        clearPendingRobotSelection()
    }


    private fun clearPendingRobotSelection() {

        pendingRobotX =
            null

        pendingRobotY =
            null


        robotSelectedCellText.text =
            "X: --   Y: --"


        setRobotDirectionButtonsEnabled(
            false
        )


        updateRobotPlacementStep(
            centreSelected = false
        )
    }


    private fun updateRobotPlacementStep(
        centreSelected: Boolean
    ) {

        if (centreSelected) {

            robotCentreLabel.text =
                "Selected Centre"

            robotCentreLabel.setTypeface(
                null,
                Typeface.NORMAL
            )

            robotCentreLabel.setTextColor(
                color(
                    R.color.odyssey_text_muted
                )
            )


            robotOrientationLabel.text =
                "Select Orientation"

            robotOrientationLabel.setTypeface(
                null,
                Typeface.BOLD
            )

            robotOrientationLabel.setTextColor(
                color(
                    R.color.odyssey_navy
                )
            )

        } else {

            robotCentreLabel.text =
                "Select Centre"

            robotCentreLabel.setTypeface(
                null,
                Typeface.BOLD
            )

            robotCentreLabel.setTextColor(
                color(
                    R.color.odyssey_navy
                )
            )


            robotOrientationLabel.text =
                "Select Orientation"

            robotOrientationLabel.setTypeface(
                null,
                Typeface.NORMAL
            )

            robotOrientationLabel.setTextColor(
                color(
                    R.color.odyssey_text_muted
                )
            )
        }
    }


    private fun setRobotDirectionButtonsEnabled(
        enabled: Boolean
    ) {

        val buttons =
            listOf(
                robotNorthButton,
                robotSouthButton,
                robotEastButton,
                robotWestButton
            )


        buttons.forEach { button ->

            button.isEnabled =
                enabled


            button.backgroundTintList =
                ColorStateList.valueOf(
                    color(
                        if (enabled) {
                            R.color.grey
                        } else {
                            R.color.control_inactive
                        }
                    )
                )


            button.setTextColor(
                color(
                    if (enabled) {
                        R.color.white
                    } else {
                        R.color.odyssey_text_muted
                    }
                )
            )
        }
    }


    // =====================================================
    // ROBOT ORIENTATION
    // =====================================================

    private fun setupRobotOrientation() {

        robotNorthButton
            .setOnClickListener {

                placeRobot(
                    Direction.NORTH
                )
            }


        robotSouthButton
            .setOnClickListener {

                placeRobot(
                    Direction.SOUTH
                )
            }


        robotEastButton
            .setOnClickListener {

                placeRobot(
                    Direction.EAST
                )
            }


        robotWestButton
            .setOnClickListener {

                placeRobot(
                    Direction.WEST
                )
            }
    }


    private fun placeRobot(
        direction: Direction
    ) {

        val x =
            pendingRobotX
                ?: return

        val y =
            pendingRobotY
                ?: return


        parentFragmentManager
            .setFragmentResult(
                RESULT_PLACE_ROBOT,
                bundleOf(
                    KEY_X to x,
                    KEY_Y to y,
                    KEY_DIRECTION to
                            direction.name
                )
            )
    }


    // =====================================================
    // MANUAL OBSTACLE PLACEMENT
    // =====================================================

    private fun setupManualObstaclePlacement() {

        setupPlacementSwitch(
            activateObstaclePlacementButton
        )


        activateObstaclePlacementButton
            .setOnCheckedChangeListener { _, isChecked ->

                if (isChecked) {

                    if (robotPlacementActive) {

                        deactivateRobotPlacement()
                    }


                    if (!obstaclePlacementActive) {

                        activateObstaclePlacement()
                    }

                } else {

                    if (obstaclePlacementActive) {

                        deactivateObstaclePlacement()
                    }
                }
            }
    }

    private fun activateObstaclePlacement() {

        obstaclePlacementActive =
            true


        clearPendingManualSelection()


        // Show both placement steps immediately.
        manualOrientationPanel.visibility =
            View.VISIBLE


        activateObstaclePlacementButton.isChecked =
            true


        updatePlacementSwitchAppearance(
            activateObstaclePlacementButton,
            true
        )


        parentFragmentManager
            .setFragmentResult(
                RESULT_PLACEMENT_MODE,
                bundleOf(
                    KEY_ACTIVE to true
                )
            )
    }


    private fun deactivateObstaclePlacement(
        notifyHome: Boolean = true
    ) {

        obstaclePlacementActive =
            false

        activateObstaclePlacementButton.isChecked =
            false


        updatePlacementSwitchAppearance(
            activateObstaclePlacementButton,
            false
        )


        clearPendingManualSelection()


        manualOrientationPanel.visibility =
            View.GONE


        if (notifyHome) {

            parentFragmentManager
                .setFragmentResult(
                    RESULT_PLACEMENT_MODE,
                    bundleOf(
                        KEY_ACTIVE to false
                    )
                )
        }
    }


    private fun clearPendingManualSelection() {

        pendingManualX =
            null

        pendingManualY =
            null


        manualSelectedCellText.text =
            "X: --   Y: --"


        setManualDirectionButtonsEnabled(
            false
        )


        updateManualPlacementStep(
            cellSelected = false
        )
    }


    private fun updateManualPlacementStep(
        cellSelected: Boolean
    ) {

        if (cellSelected) {

            manualCellLabel.text =
                "Selected Cell"

            manualCellLabel.setTypeface(
                null,
                Typeface.NORMAL
            )

            manualCellLabel.setTextColor(
                color(
                    R.color.odyssey_text_muted
                )
            )


            manualOrientationLabel.text =
                "Select Orientation"

            manualOrientationLabel.setTypeface(
                null,
                Typeface.BOLD
            )

            manualOrientationLabel.setTextColor(
                color(
                    R.color.odyssey_navy
                )
            )

        } else {

            manualCellLabel.text =
                "Select Cell"

            manualCellLabel.setTypeface(
                null,
                Typeface.BOLD
            )

            manualCellLabel.setTextColor(
                color(
                    R.color.odyssey_navy
                )
            )


            manualOrientationLabel.text =
                "Select Orientation"

            manualOrientationLabel.setTypeface(
                null,
                Typeface.NORMAL
            )

            manualOrientationLabel.setTextColor(
                color(
                    R.color.odyssey_text_muted
                )
            )
        }
    }


    private fun setManualDirectionButtonsEnabled(
        enabled: Boolean
    ) {

        val buttons =
            listOf(
                manualNorthButton,
                manualSouthButton,
                manualEastButton,
                manualWestButton
            )


        buttons.forEach { button ->

            button.isEnabled =
                enabled


            button.backgroundTintList =
                ColorStateList.valueOf(
                    color(
                        if (enabled) {
                            R.color.grey
                        } else {
                            R.color.control_inactive
                        }
                    )
                )


            button.setTextColor(
                color(
                    if (enabled) {
                        R.color.white
                    } else {
                        R.color.odyssey_text_muted
                    }
                )
            )
        }
    }


    // =====================================================
    // MANUAL OBSTACLE ORIENTATION
    // =====================================================

    private fun setupManualOrientation() {

        manualNorthButton
            .setOnClickListener {

                placeManualObstacle(
                    Direction.NORTH
                )
            }


        manualSouthButton
            .setOnClickListener {

                placeManualObstacle(
                    Direction.SOUTH
                )
            }


        manualEastButton
            .setOnClickListener {

                placeManualObstacle(
                    Direction.EAST
                )
            }


        manualWestButton
            .setOnClickListener {

                placeManualObstacle(
                    Direction.WEST
                )
            }
    }


    private fun placeManualObstacle(
        direction: Direction
    ) {

        val x =
            pendingManualX
                ?: return

        val y =
            pendingManualY
                ?: return


        parentFragmentManager
            .setFragmentResult(
                RESULT_PLACE_OBSTACLE,
                bundleOf(
                    KEY_X to x,
                    KEY_Y to y,
                    KEY_DIRECTION to
                            direction.name
                )
            )


        // Keep manual obstacle placement active so the
        // user can immediately place another obstacle.
        clearPendingManualSelection()
    }


    private fun setupPlacementSwitch(
        switch: SwitchCompat
    ) {

        val states =
            arrayOf(
                intArrayOf(
                    android.R.attr.state_checked
                ),
                intArrayOf()
            )


        // White thumb in both states.
        switch.thumbTintList =
            ColorStateList(
                states,
                intArrayOf(
                    color(R.color.white),
                    color(R.color.white)
                )
            )


        // Green when active, grey when inactive.
        switch.trackTintList =
            ColorStateList(
                states,
                intArrayOf(
                    color(R.color.control_active),
                    color(R.color.control_inactive)
                )
            )


        updatePlacementSwitchAppearance(
            switch,
            switch.isChecked
        )
    }


    private fun updatePlacementSwitchAppearance(
        switch: SwitchCompat,
        active: Boolean
    ) {

        switch.text =
            if (active) {
                "Activated"
            } else {
                "Deactivated"
            }


        switch.setTextColor(
            color(
                if (active) {
                    R.color.control_active
                } else {
                    R.color.odyssey_text_muted
                }
            )
        )
    }


    // =====================================================
    // INPUT OBSTACLE
    // =====================================================

    private fun setupCoordinatePickers() {

        xPicker.minValue =
            0

        xPicker.maxValue =
            19

        xPicker.value =
            0

        xPicker.wrapSelectorWheel =
            false


        yPicker.minValue =
            0

        yPicker.maxValue =
            19

        yPicker.value =
            0

        yPicker.wrapSelectorWheel =
            false
    }


    private fun setupInputOrientation() {

        inputNorthButton
            .setOnClickListener {

                selectInputDirection(
                    Direction.NORTH
                )
            }


        inputEastButton
            .setOnClickListener {

                selectInputDirection(
                    Direction.EAST
                )
            }


        inputSouthButton
            .setOnClickListener {

                selectInputDirection(
                    Direction.SOUTH
                )
            }


        inputWestButton
            .setOnClickListener {

                selectInputDirection(
                    Direction.WEST
                )
            }
    }


    private fun selectInputDirection(
        direction: Direction
    ) {

        selectedInputDirection =
            direction


        setUnselectedDirection(
            inputNorthButton
        )

        setUnselectedDirection(
            inputEastButton
        )

        setUnselectedDirection(
            inputSouthButton
        )

        setUnselectedDirection(
            inputWestButton
        )


        when (direction) {

            Direction.NORTH -> {

                setSelectedDirection(
                    inputNorthButton
                )
            }


            Direction.EAST -> {

                setSelectedDirection(
                    inputEastButton
                )
            }


            Direction.SOUTH -> {

                setSelectedDirection(
                    inputSouthButton
                )
            }


            Direction.WEST -> {

                setSelectedDirection(
                    inputWestButton
                )
            }
        }
    }


    private fun setupInputPlacement() {

        placeObstacleButton
            .setOnClickListener {

                parentFragmentManager
                    .setFragmentResult(
                        RESULT_PLACE_OBSTACLE,
                        bundleOf(
                            KEY_X to
                                    xPicker.value,

                            KEY_Y to
                                    yPicker.value,

                            KEY_DIRECTION to
                                    selectedInputDirection.name
                        )
                    )
            }
    }


    // =====================================================
    // RESULTS FROM HOME / GRID
    // =====================================================

    private fun setupFragmentResults() {

        // -------------------------------------------------
        // MANUAL OBSTACLE CELL SELECTED
        // -------------------------------------------------

        parentFragmentManager
            .setFragmentResultListener(
                RESULT_GRID_SELECTION,
                viewLifecycleOwner
            ) { _, bundle ->


                if (
                    !obstaclePlacementActive ||
                    selectedTab !=
                    ConfigurationTab.PLACE_OBSTACLE
                ) {
                    return@setFragmentResultListener
                }


                val valid =
                    bundle.getBoolean(
                        KEY_VALID
                    )


                if (!valid) {

                    clearPendingManualSelection()

                    return@setFragmentResultListener
                }


                val x =
                    bundle.getInt(
                        KEY_X
                    )

                val y =
                    bundle.getInt(
                        KEY_Y
                    )


                pendingManualX =
                    x

                pendingManualY =
                    y


                manualSelectedCellText.text =
                    "X: $x   Y: $y"


                updateManualPlacementStep(
                    cellSelected = true
                )


                setManualDirectionButtonsEnabled(
                    true
                )
            }


        // -------------------------------------------------
        // ROBOT GRID RESULT
        // -------------------------------------------------

        parentFragmentManager
            .setFragmentResultListener(
                RESULT_ROBOT_GRID_SELECTION,
                viewLifecycleOwner
            ) { _, bundle ->


                if (
                    !robotPlacementActive ||
                    selectedTab !=
                    ConfigurationTab.PLACE_ROBOT
                ) {
                    return@setFragmentResultListener
                }


                val valid =
                    bundle.getBoolean(
                        KEY_VALID
                    )


                if (!valid) {

                    clearPendingRobotSelection()

                    return@setFragmentResultListener
                }


                val x =
                    bundle.getInt(
                        KEY_X
                    )

                val y =
                    bundle.getInt(
                        KEY_Y
                    )


                onRobotGridSelected(
                    x,
                    y
                )
            }


        // -------------------------------------------------
        // OBSTACLE PLACE RESULT
        // -------------------------------------------------

        parentFragmentManager
            .setFragmentResultListener(
                RESULT_OBSTACLE_PLACE_STATUS,
                viewLifecycleOwner
            ) { _, bundle ->


                val success =
                    bundle.getBoolean(
                        KEY_SUCCESS
                    )


                if (!success) {

                    // No UI action required.
                }
            }


        // -------------------------------------------------
        // ROBOT PLACE RESULT
        // -------------------------------------------------

        parentFragmentManager
            .setFragmentResultListener(
                RESULT_ROBOT_PLACE_STATUS,
                viewLifecycleOwner
            ) { _, bundle ->


                val success =
                    bundle.getBoolean(
                        KEY_SUCCESS
                    )


                if (success) {

                    // Only one robot is placed,
                    // so robot placement ends.
                    deactivateRobotPlacement()
                }
            }


        // -------------------------------------------------
        // USER LEFT CONFIGURATION
        // -------------------------------------------------

        parentFragmentManager
            .setFragmentResultListener(
                RESULT_FORCE_PLACEMENT_OFF,
                viewLifecycleOwner
            ) { _, _ ->


                deactivateObstaclePlacement(
                    notifyHome = false
                )


                deactivateRobotPlacement(
                    notifyHome = false
                )
            }
    }


    // =====================================================
    // TAB STYLE
    // =====================================================

    private fun setSelectedTab(
        button: Button
    ) {

        button.backgroundTintList =
            ColorStateList.valueOf(
                color(
                    R.color.odyssey_bronze_light
                )
            )


        button.setTextColor(
            color(
                R.color.white
            )
        )
    }


    private fun setUnselectedTab(
        button: Button
    ) {

        button.backgroundTintList =
            ColorStateList.valueOf(
                color(
                    R.color.unselected
                )
            )


        button.setTextColor(
            color(
                R.color.black
            )
        )
    }


    // =====================================================
    // INPUT DIRECTION STYLE
    // =====================================================

    private fun setSelectedDirection(
        button: Button
    ) {

        button.backgroundTintList =
            ColorStateList.valueOf(
                color(
                    R.color.grey
                )
            )


        button.setTextColor(
            color(
                R.color.white
            )
        )
    }


    private fun setUnselectedDirection(
        button: Button
    ) {

        button.backgroundTintList =
            ColorStateList.valueOf(
                color(
                    R.color.control_inactive
                )
            )


        button.setTextColor(
            color(
                R.color.odyssey_text_muted
            )
        )
    }


    // =====================================================
    // MAP ACTIONS
    // =====================================================

    private fun setupMapActions() {

        undoMapButton
            .setOnClickListener {


                if (robotPlacementActive) {

                    deactivateRobotPlacement()
                }


                if (obstaclePlacementActive) {

                    deactivateObstaclePlacement()
                }


                (parentFragment as? HomeFragment)
                    ?.undoLastPlacement()
            }


        saveMapButton
            .setOnClickListener {

                deactivatePlacementModes()


                val saved =
                    (parentFragment as? HomeFragment)
                        ?.saveCurrentLayout()
                        ?: false


                Toast.makeText(
                    requireContext(),
                    if (saved) {
                        "Layout saved"
                    } else {
                        "Unable to save layout"
                    },
                    Toast.LENGTH_SHORT
                ).show()
            }


        useMapButton
            .setOnClickListener {

                deactivatePlacementModes()


                val loaded =
                    (parentFragment as? HomeFragment)
                        ?.useSavedLayout()
                        ?: false


                Toast.makeText(
                    requireContext(),
                    if (loaded) {
                        "Saved layout loaded"
                    } else {
                        "No saved layout found"
                    },
                    Toast.LENGTH_SHORT
                ).show()
            }


        resetMapButton
            .setOnClickListener {


                if (robotPlacementActive) {

                    deactivateRobotPlacement()
                }


                if (obstaclePlacementActive) {

                    deactivateObstaclePlacement()
                }


                (parentFragment as? HomeFragment)
                    ?.resetArena()
            }


        sendMapButton
            .setOnClickListener {


                if (robotPlacementActive) {

                    deactivateRobotPlacement()
                }


                if (obstaclePlacementActive) {

                    deactivateObstaclePlacement()
                }


                val sent =
                    (parentFragment as? HomeFragment)
                        ?.sendFullMap()
                        ?: false


                Toast.makeText(
                    requireContext(),
                    if (sent) {
                        "Map sent. Preparing route before Task 1 Start."
                    } else {
                        "Unable to send map. Connect Bluetooth and place a robot and obstacles."
                    },
                    Toast.LENGTH_SHORT
                ).show()
            }
    }


    private fun deactivatePlacementModes() {

        if (robotPlacementActive) {
            deactivateRobotPlacement()
        }


        if (obstaclePlacementActive) {
            deactivateObstaclePlacement()
        }
    }


    // =====================================================
    // COLOR
    // =====================================================

    private fun color(
        resource: Int
    ): Int {

        return ContextCompat.getColor(
            requireContext(),
            resource
        )
    }


    // =====================================================
    // RESULT KEYS
    // =====================================================

    companion object {

        const val RESULT_PLACEMENT_MODE =
            "configuration_placement_mode"

        const val RESULT_GRID_SELECTION =
            "configuration_grid_selection"

        const val RESULT_PLACE_OBSTACLE =
            "configuration_place_obstacle"

        const val RESULT_OBSTACLE_PLACE_STATUS =
            "configuration_obstacle_place_status"


        const val RESULT_ROBOT_PLACEMENT_MODE =
            "configuration_robot_placement_mode"

        const val RESULT_ROBOT_GRID_SELECTION =
            "configuration_robot_grid_selection"

        const val RESULT_PLACE_ROBOT =
            "configuration_place_robot"

        const val RESULT_ROBOT_PLACE_STATUS =
            "configuration_robot_place_status"


        const val RESULT_FORCE_PLACEMENT_OFF =
            "configuration_force_placement_off"


        const val KEY_ACTIVE =
            "active"

        const val KEY_VALID =
            "valid"

        const val KEY_X =
            "x"

        const val KEY_Y =
            "y"

        const val KEY_DIRECTION =
            "direction"

        const val KEY_SUCCESS =
            "success"
    }
}
