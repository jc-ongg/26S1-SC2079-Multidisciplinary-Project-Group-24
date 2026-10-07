package com.example.mdpgroup24.fragments

import android.graphics.Typeface
import android.os.Bundle
import android.view.MotionEvent
import android.view.View
import android.widget.LinearLayout
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.core.os.bundleOf
import androidx.fragment.app.Fragment
import androidx.viewpager2.widget.ViewPager2
import com.example.mdpgroup24.R
import com.example.mdpgroup24.adapters.SubTabPagerAdapter
import com.example.mdpgroup24.views.ArenaGridView
import kotlin.math.abs
import android.bluetooth.BluetoothDevice
import com.example.mdpgroup24.bluetooth.BluetoothConnectionService
import com.example.mdpgroup24.bluetooth.BluetoothSession
import android.graphics.Color
import androidx.appcompat.app.AlertDialog
import android.content.res.ColorStateList
import android.widget.Button
import org.json.JSONArray
import org.json.JSONObject

class HomeFragment :
    Fragment(R.layout.fragment_home),
    BluetoothSession.Listener {

    private lateinit var arenaGridView:
            ArenaGridView


    private val protocolReceiveBuffer =
        StringBuilder()

    private lateinit var vesselStatus:
            TextView

    private lateinit var vesselTaskStatus:
            TextView

    override fun onViewCreated(
        view: View,
        savedInstanceState: Bundle?
    ) {

        super.onViewCreated(
            view,
            savedInstanceState
        )


        arenaGridView =
            view.findViewById(
                R.id.arenaGridView
            )

        vesselStatus =
            view.findViewById(
                R.id.vesselStatus
            )

        vesselTaskStatus =
            view.findViewById(
                R.id.vesselTaskStatus
            )

        BluetoothSession.initialize(
            requireContext()
        )

        BluetoothSession.addListener(
            this
        )

        updateBluetoothStatus(
            BluetoothSession.isConnected
        )

        setupArenaInteraction()


        // =====================================================
        // SUBTABS
        // =====================================================

        val tabConfiguration =
            view.findViewById<TextView>(
                R.id.tabConfiguration
            )

        val tabControl =
            view.findViewById<TextView>(
                R.id.tabControl
            )

        val tabChat =
            view.findViewById<TextView>(
                R.id.tabChat
            )

        val tabChallenge =
            view.findViewById<TextView>(
                R.id.tabChallenge
            )


        val subTabBar =
            view.findViewById<LinearLayout>(
                R.id.subTabBar
            )


        val viewPager =
            view.findViewById<ViewPager2>(
                R.id.subTabViewPager
            )


        viewPager.adapter =
            SubTabPagerAdapter(
                this
            )


        // Content itself cannot swipe.
        viewPager.isUserInputEnabled =
            false


        // =====================================================
        // TAB TAPS
        // =====================================================

        tabConfiguration
            .setOnClickListener {

                viewPager.currentItem =
                    0
            }


        tabControl
            .setOnClickListener {

                viewPager.currentItem =
                    1
            }


        tabChat
            .setOnClickListener {

                viewPager.currentItem =
                    2
            }


        tabChallenge
            .setOnClickListener {

                viewPager.currentItem =
                    3
            }


        // =====================================================
        // SWIPE ON SUBTAB BAR
        // =====================================================

        var startX =
            0f


        val swipeListener =
            View.OnTouchListener {
                    touchedView,
                    event ->


                when (
                    event.actionMasked
                ) {

                    MotionEvent.ACTION_DOWN -> {

                        startX =
                            event.x


                        touchedView.parent
                            .requestDisallowInterceptTouchEvent(
                                true
                            )
                    }


                    MotionEvent.ACTION_MOVE -> {

                        touchedView.parent
                            .requestDisallowInterceptTouchEvent(
                                true
                            )
                    }


                    MotionEvent.ACTION_UP -> {

                        val differenceX =
                            event.x -
                                    startX


                        if (
                            abs(
                                differenceX
                            ) > 80
                        ) {

                            if (
                                differenceX < 0
                            ) {

                                if (
                                    viewPager.currentItem <
                                    3
                                ) {

                                    viewPager.currentItem +=
                                        1
                                }

                            } else {

                                if (
                                    viewPager.currentItem >
                                    0
                                ) {

                                    viewPager.currentItem -=
                                        1
                                }
                            }

                        } else {

                            touchedView.performClick()
                        }


                        touchedView.parent
                            .requestDisallowInterceptTouchEvent(
                                false
                            )
                    }


                    MotionEvent.ACTION_CANCEL -> {

                        touchedView.parent
                            .requestDisallowInterceptTouchEvent(
                                false
                            )
                    }
                }


                true
            }


        subTabBar
            .setOnTouchListener(
                swipeListener
            )

        tabConfiguration
            .setOnTouchListener(
                swipeListener
            )

        tabControl
            .setOnTouchListener(
                swipeListener
            )

        tabChat
            .setOnTouchListener(
                swipeListener
            )

        tabChallenge
            .setOnTouchListener(
                swipeListener
            )


        // =====================================================
        // TAB APPEARANCE
        // =====================================================

        viewPager
            .registerOnPageChangeCallback(

                object :
                    ViewPager2.OnPageChangeCallback() {

                    override fun onPageSelected(
                        position: Int
                    ) {

                        super.onPageSelected(
                            position
                        )


                        updateTabAppearance(
                            position,
                            tabConfiguration,
                            tabControl,
                            tabChat,
                            tabChallenge
                        )


                        // If user leaves Configuration,
                        // stop placement mode.
                        if (
                            position != 0
                        ) {

                            arenaGridView
                                .setSelectionMode(
                                    ArenaGridView
                                        .SelectionMode
                                        .NONE
                                )


                            childFragmentManager
                                .setFragmentResult(
                                    ConfigurationFragment
                                        .RESULT_FORCE_PLACEMENT_OFF,
                                    Bundle()
                                )
                        }
                    }
                }
            )


        // Configuration selected by default
        updateTabAppearance(
            0,
            tabConfiguration,
            tabControl,
            tabChat,
            tabChallenge
        )
    }


    // =====================================================
    // ARENA / CONFIGURATION COMMUNICATION
    // =====================================================

    private fun setupArenaInteraction() {

        // =====================================================
        // OBSTACLE PLACEMENT MODE
        // =====================================================

        childFragmentManager
            .setFragmentResultListener(
                ConfigurationFragment
                    .RESULT_PLACEMENT_MODE,
                viewLifecycleOwner
            ) { _, bundle ->

                val active =
                    bundle.getBoolean(
                        ConfigurationFragment
                            .KEY_ACTIVE
                    )


                if (active) {

                    arenaGridView
                        .setSelectionMode(
                            ArenaGridView
                                .SelectionMode
                                .OBSTACLE
                        )

                } else {

                    arenaGridView
                        .setSelectionMode(
                            ArenaGridView
                                .SelectionMode
                                .NONE
                        )
                }
            }


        // =====================================================
        // PLACE OBSTACLE
        // =====================================================

        childFragmentManager
            .setFragmentResultListener(
                ConfigurationFragment
                    .RESULT_PLACE_OBSTACLE,
                viewLifecycleOwner
            ) { _, bundle ->

                val x =
                    bundle.getInt(
                        ConfigurationFragment
                            .KEY_X
                    )

                val y =
                    bundle.getInt(
                        ConfigurationFragment
                            .KEY_Y
                    )


                val directionName =
                    bundle.getString(
                        ConfigurationFragment
                            .KEY_DIRECTION
                    )
                        ?: return@setFragmentResultListener


                val direction =
                    try {

                        ArenaGridView.Direction
                            .valueOf(
                                directionName
                            )

                    } catch (
                        exception: IllegalArgumentException
                    ) {

                        return@setFragmentResultListener
                    }


                val success =
                    arenaGridView
                        .addObstacle(
                            x,
                            y,
                            direction
                        )

                if (success) {

                    val obstacle =
                        arenaGridView
                            .getObstacleAt(
                                x,
                                y
                            )


                    if (obstacle != null) {

                        sendObstacleUpdate(
                            obstacle
                        )
                    }
                }


                childFragmentManager
                    .setFragmentResult(
                        ConfigurationFragment
                            .RESULT_OBSTACLE_PLACE_STATUS,

                        bundleOf(
                            ConfigurationFragment
                                .KEY_SUCCESS to
                                    success
                        )
                    )
            }


        // =====================================================
        // PLACE ROBOT
        // =====================================================

        childFragmentManager
            .setFragmentResultListener(
                ConfigurationFragment
                    .RESULT_PLACE_ROBOT,
                viewLifecycleOwner
            ) { _, bundle ->

                val x =
                    bundle.getInt(
                        ConfigurationFragment
                            .KEY_X
                    )

                val y =
                    bundle.getInt(
                        ConfigurationFragment
                            .KEY_Y
                    )


                val directionName =
                    bundle.getString(
                        ConfigurationFragment
                            .KEY_DIRECTION
                    )
                        ?: return@setFragmentResultListener


                val direction =
                    try {

                        ArenaGridView.Direction
                            .valueOf(
                                directionName
                            )

                    } catch (
                        exception: IllegalArgumentException
                    ) {

                        return@setFragmentResultListener
                    }


                val success =
                    arenaGridView
                        .setRobot(
                            x,
                            y,
                            direction
                        )
                if (success) {

                    updateVesselInformation()

                    arenaGridView
                        .getRobot()
                        ?.let { robot ->

                            sendRobotConfiguration(
                                robot
                            )
                        }
                }


                childFragmentManager
                    .setFragmentResult(
                        ConfigurationFragment
                            .RESULT_ROBOT_PLACE_STATUS,

                        bundleOf(
                            ConfigurationFragment
                                .KEY_SUCCESS to
                                    success
                        )
                    )
            }


        // =====================================================
        // GRID SELECTION
        // =====================================================

        arenaGridView.setSelectionListener(

            object : ArenaGridView.SelectionListener {

                override fun onGridCellSelected(
                    mode: ArenaGridView.SelectionMode,
                    x: Int,
                    y: Int
                ) {

                    when (mode) {

                        ArenaGridView.SelectionMode.OBSTACLE -> {

                            childFragmentManager
                                .setFragmentResult(
                                    ConfigurationFragment
                                        .RESULT_GRID_SELECTION,

                                    bundleOf(
                                        ConfigurationFragment
                                            .KEY_VALID to
                                                true,

                                        ConfigurationFragment
                                            .KEY_X to
                                                x,

                                        ConfigurationFragment
                                            .KEY_Y to
                                                y
                                    )
                                )
                        }


                        ArenaGridView.SelectionMode.ROBOT -> {

                            getConfigurationFragment()
                                ?.onRobotGridSelected(
                                    x,
                                    y
                                )
                        }


                        ArenaGridView.SelectionMode.NONE -> {
                            // Nothing
                        }
                    }
                }


                override fun onInvalidGridSelection(
                    mode: ArenaGridView.SelectionMode,
                    x: Int,
                    y: Int
                ) {

                    when (mode) {

                        ArenaGridView.SelectionMode.OBSTACLE -> {

                            childFragmentManager
                                .setFragmentResult(
                                    ConfigurationFragment
                                        .RESULT_GRID_SELECTION,

                                    bundleOf(
                                        ConfigurationFragment
                                            .KEY_VALID to
                                                false
                                    )
                                )
                        }


                        ArenaGridView.SelectionMode.ROBOT -> {

                            getConfigurationFragment()
                                ?.onInvalidRobotGridSelection()
                        }


                        ArenaGridView.SelectionMode.NONE -> {
                            // Nothing
                        }
                    }
                }
            }
        )

        arenaGridView.setObstacleInteractionListener(

            object :
                ArenaGridView.ObstacleInteractionListener {


                override fun onObstacleTapped(
                    obstacle: ArenaGridView.Obstacle
                ) {

                    showObstacleEditor(
                        obstacle
                    )
                }


                override fun onObstacleMoved(
                    obstacle: ArenaGridView.Obstacle
                ) {

                    sendObstacleUpdate(
                        obstacle
                    )
                }


                override fun onObstacleRemoved(
                    obstacle: ArenaGridView.Obstacle
                ) {

                    sendObstacleRemoval(
                        obstacle
                    )
                }
            }
        )
    }


    // =====================================================
// OBSTACLE EDITOR
// =====================================================

    private fun showObstacleEditor(
        obstacle: ArenaGridView.Obstacle
    ) {

        val dialogView =
            layoutInflater.inflate(
                R.layout.dialog_edit_obstacle,
                null
            )


        val title =
            dialogView.findViewById<TextView>(
                R.id.dialogObstacleTitle
            )

        val coordinates =
            dialogView.findViewById<TextView>(
                R.id.dialogObstacleCoordinates
            )


        val northButton =
            dialogView.findViewById<Button>(
                R.id.dialogNorth
            )

        val eastButton =
            dialogView.findViewById<Button>(
                R.id.dialogEast
            )

        val southButton =
            dialogView.findViewById<Button>(
                R.id.dialogSouth
            )

        val westButton =
            dialogView.findViewById<Button>(
                R.id.dialogWest
            )


        val removeButton =
            dialogView.findViewById<Button>(
                R.id.dialogRemove
            )

        removeButton.backgroundTintList =
            ColorStateList.valueOf(
                ContextCompat.getColor(
                    requireContext(),
                    R.color.status_red
                )
            )

        removeButton.setTextColor(
            ContextCompat.getColor(
                requireContext(),
                R.color.white
            )
        )


        val saveButton =
            dialogView.findViewById<Button>(
                R.id.dialogSave
            )

        saveButton.backgroundTintList =
            ColorStateList.valueOf(
                ContextCompat.getColor(
                    requireContext(),
                    R.color.control_active
                )
            )

        saveButton.setTextColor(
            ContextCompat.getColor(
                requireContext(),
                R.color.white
            )
        )


        title.text =
            if (obstacle.targetId != null) {

                "Obstacle ${obstacle.id}   (Target = ${obstacle.targetId})"

            } else {

                "Obstacle ${obstacle.id}"
            }


        coordinates.text =
            "X: ${obstacle.x}    Y: ${obstacle.y}"


        var selectedDirection =
            obstacle.direction


        val directionButtons =
            mapOf(
                ArenaGridView.Direction.NORTH to northButton,
                ArenaGridView.Direction.EAST to eastButton,
                ArenaGridView.Direction.SOUTH to southButton,
                ArenaGridView.Direction.WEST to westButton
            )


        // =====================================================
        // DIRECTION APPEARANCE
        // =====================================================

        fun updateDirectionAppearance() {

            directionButtons.forEach {
                    (direction, button) ->

                if (
                    direction ==
                    selectedDirection
                ) {

                    // Selected orientation:
                    // bronze background + navy text.
                    button.backgroundTintList =
                        ColorStateList.valueOf(
                            ContextCompat.getColor(
                                requireContext(),
                                R.color.grey
                            )
                        )

                    button.setTextColor(
                        ContextCompat.getColor(
                            requireContext(),
                            R.color.white
                        )
                    )

                } else {

                    // Unselected orientation:
                    // light grey background + muted text.
                    button.backgroundTintList =
                        ColorStateList.valueOf(
                            ContextCompat.getColor(
                                requireContext(),
                                R.color.control_inactive
                            )
                        )

                    button.setTextColor(
                        ContextCompat.getColor(
                            requireContext(),
                            R.color.odyssey_text_muted
                        )
                    )
                }
            }
        }

        // Show existing orientation immediately.
        updateDirectionAppearance()


        northButton.setOnClickListener {

            selectedDirection =
                ArenaGridView.Direction.NORTH

            updateDirectionAppearance()
        }


        eastButton.setOnClickListener {

            selectedDirection =
                ArenaGridView.Direction.EAST

            updateDirectionAppearance()
        }


        southButton.setOnClickListener {

            selectedDirection =
                ArenaGridView.Direction.SOUTH

            updateDirectionAppearance()
        }


        westButton.setOnClickListener {

            selectedDirection =
                ArenaGridView.Direction.WEST

            updateDirectionAppearance()
        }


        // =====================================================
        // CREATE DIALOG
        // =====================================================

        val dialog =
            AlertDialog.Builder(
                requireContext()
            )
                .setView(
                    dialogView
                )
                .create()


        // =====================================================
        // ACTIONS
        // =====================================================

        saveButton.setOnClickListener {

            val success =
                arenaGridView
                    .updateObstacleDirection(
                        obstacle.id,
                        selectedDirection
                    )


            if (success) {

                val updatedObstacle =
                    arenaGridView
                        .getObstacleById(
                            obstacle.id
                        )


                if (updatedObstacle != null) {

                    sendObstacleUpdate(
                        updatedObstacle
                    )
                }
            }


            dialog.dismiss()
        }

        removeButton.setOnClickListener {

            val success =
                arenaGridView
                    .removeObstacle(
                        obstacle.id
                    )


            if (success) {

                sendObstacleRemoval(
                    obstacle
                )
            }


            dialog.dismiss()
        }



        dialog.show()
    }

    // =====================================================
    // TAB APPEARANCE
    // =====================================================

    private fun updateTabAppearance(
        selectedPage: Int,
        tabConfiguration: TextView,
        tabControl: TextView,
        tabChat: TextView,
        tabChallenge: TextView
    ) {

        val tabs =
            listOf(
                tabConfiguration,
                tabControl,
                tabChat,
                tabChallenge
            )


        val selectedTextColor =
            ContextCompat.getColor(
                requireContext(),
                R.color.odyssey_cream
            )


        val unselectedTextColor =
            ContextCompat.getColor(
                requireContext(),
                R.color.black
            )


        tabs.forEach { tab ->

            tab.setBackgroundResource(
                R.drawable.bg_sub_tab_unselected
            )


            tab.setTextColor(
                unselectedTextColor
            )


            tab.setTypeface(
                null,
                Typeface.NORMAL
            )
        }


        tabs[selectedPage]
            .setBackgroundResource(
                R.drawable.bg_sub_tab_selected
            )


        tabs[selectedPage]
            .setTextColor(
                selectedTextColor
            )


        tabs[selectedPage]
            .setTypeface(
                null,
                Typeface.BOLD
            )
    }



    fun updateRobotFromExternal(
        x: Int,
        y: Int,
        direction: ArenaGridView.Direction
    ): Boolean {

        val success =
            arenaGridView
                .updateRobotFromExternal(
                    x,
                    y,
                    direction
                )


        if (success) {

            updateVesselInformation()
        }


        return success
    }

    // =====================================================
// ROBOT CONTROL
// =====================================================

    fun moveRobotForward(): Boolean {

        val success =
            arenaGridView.moveRobotForward()

        if (success) {
            updateVesselInformation()
        }

        return success
    }

    fun moveRobotBackward(): Boolean {

        val success =
            arenaGridView.moveRobotBackward()

        if (success) {
            updateVesselInformation()
        }

        return success
    }


    fun rotateRobotLeft(): Boolean {

        val success =
            arenaGridView.rotateRobotLeft()

        if (success) {
            updateVesselInformation()
        }

        return success
    }


    fun rotateRobotRight(): Boolean {

        val success =
            arenaGridView.rotateRobotRight()

        if (success) {
            updateVesselInformation()
        }

        return success
    }


    private fun updateVesselInformation() {

        val coordinates =
            view?.findViewById<TextView>(
                R.id.vesselCoordinates
            )


        val direction =
            view?.findViewById<TextView>(
                R.id.vesselDirection
            )


        val robot =
            arenaGridView.getRobot()


        if (robot == null) {

            coordinates?.text =
                "X: --    Y: --"

            direction?.text =
                "--"

            return
        }


        coordinates?.text =
            "X: ${robot.x}    Y: ${robot.y}"


        direction?.text =
            robot.direction.name
    }

    // =====================================================
    // GRID MODE
    // =====================================================

    fun setArenaSelectionMode(
        mode: ArenaGridView.SelectionMode
    ) {

        arenaGridView
            .setSelectionMode(
                mode
            )
    }


    // =====================================================
// ARENA ACTIONS
// =====================================================

    fun undoLastPlacement(): Boolean {

        val result =
            arenaGridView
                .undoLastPlacement()
                ?: return false


        updateVesselInformation()


        when (result) {

            is ArenaGridView
            .UndoResult
            .ObstacleRemoved -> {

                sendObstacleRemoval(
                    result.obstacle
                )
            }


            is ArenaGridView
            .UndoResult
            .RobotRemoved -> {

                sendRobotRemoval(
                    result.robot
                )
            }
        }


        return true
    }


    fun resetArena() {

        arenaGridView
            .resetArena()


        updateVesselInformation()


        sendProtocolMessage(
            "MAP"
        )
    }

    override fun onManualCompleted(command: String) {
        when (command) {
            "F" -> moveRobotForward()
            "B" -> moveRobotBackward()
            "L" -> rotateRobotLeft()
            "R" -> rotateRobotRight()
        }
    }


    fun saveCurrentLayout(): Boolean {

        val layout =
            JSONObject()


        val currentRobot =
            arenaGridView.getRobot()


        if (currentRobot == null) {

            layout.put(
                "robot",
                JSONObject.NULL
            )

        } else {

            layout.put(
                "robot",
                JSONObject().apply {
                    put("x", currentRobot.x)
                    put("y", currentRobot.y)
                    put(
                        "direction",
                        currentRobot.direction.name
                    )
                }
            )
        }


        val obstacles =
            JSONArray()


        arenaGridView
            .getObstacles()
            .sortedBy {
                it.id
            }
            .forEach { obstacle ->

                obstacles.put(
                    JSONObject().apply {
                        put("id", obstacle.id)
                        put("x", obstacle.x)
                        put("y", obstacle.y)
                        put(
                            "direction",
                            obstacle.direction.name
                        )
                    }
                )
            }


        layout.put(
            "obstacles",
            obstacles
        )


        requireContext()
            .getSharedPreferences(
                SAVED_LAYOUT_PREFERENCES,
                android.content.Context.MODE_PRIVATE
            )
            .edit()
            .putString(
                SAVED_LAYOUT_KEY,
                layout.toString()
            )
            .apply()


        return true
    }


    fun useSavedLayout(): Boolean {

        val savedLayout =
            requireContext()
                .getSharedPreferences(
                    SAVED_LAYOUT_PREFERENCES,
                    android.content.Context.MODE_PRIVATE
                )
                .getString(
                    SAVED_LAYOUT_KEY,
                    null
                )
                ?: return false


        return try {

            val layout =
                JSONObject(savedLayout)


            val robotJson =
                layout.optJSONObject(
                    "robot"
                )


            val savedRobot =
                robotJson?.let {
                    ArenaGridView.Robot(
                        x = it.getInt("x"),
                        y = it.getInt("y"),
                        direction =
                            ArenaGridView.Direction.valueOf(
                                it.getString(
                                    "direction"
                                )
                            )
                    )
                }


            val obstacleJson =
                layout.getJSONArray(
                    "obstacles"
                )


            val savedObstacles =
                buildList {

                    for (
                    index in
                    0 until obstacleJson.length()
                    ) {

                        val obstacle =
                            obstacleJson.getJSONObject(
                                index
                            )


                        add(
                            ArenaGridView.Obstacle(
                                id = obstacle.getInt("id"),
                                x = obstacle.getInt("x"),
                                y = obstacle.getInt("y"),
                                direction =
                                    ArenaGridView.Direction.valueOf(
                                        obstacle.getString(
                                            "direction"
                                        )
                                    )
                            )
                        )
                    }
                }


            val loaded =
                arenaGridView.replaceLayout(
                    savedRobot,
                    savedObstacles
                )


            if (loaded) {
                BluetoothSession.invalidatePreparation()
                updateVesselInformation()
            }


            loaded

        } catch (
            _: Exception
        ) {
            false
        }
    }


    // =====================================================
    // GET CONFIGURATION FRAGMENT
    // =====================================================

    private fun getConfigurationFragment():
            ConfigurationFragment? {

        return childFragmentManager
            .fragments
            .filterIsInstance<ConfigurationFragment>()
            .firstOrNull()
    }

    // =====================================================
// OBSTACLE BLUETOOTH
// =====================================================

// =====================================================
// MAP BLUETOOTH PROTOCOL
// =====================================================

    private fun directionCode(
        direction: ArenaGridView.Direction
    ): String {

        return when (direction) {

            ArenaGridView.Direction.NORTH ->
                "N"

            ArenaGridView.Direction.EAST ->
                "E"

            ArenaGridView.Direction.SOUTH ->
                "S"

            ArenaGridView.Direction.WEST ->
                "W"
        }
    }


    private fun sendProtocolMessage(
        message: String
    ): Boolean {

        BluetoothSession.invalidatePreparation()

        if (!BluetoothSession.isConnected) {
            return false
        }

        return BluetoothSession.send(
            "//$message//"
        )
    }


    private fun sendObstacleUpdate(
        obstacle: ArenaGridView.Obstacle
    ): Boolean {

        val message =
            "OBSTACLE," +
                    "${obstacle.id}," +
                    "${obstacle.x}," +
                    "${obstacle.y}," +
                    directionCode(
                        obstacle.direction
                    )

        return sendProtocolMessage(
            message
        )
    }


    private fun sendObstacleRemoval(
        obstacle: ArenaGridView.Obstacle
    ): Boolean {

        val message =
            "OBSTACLE," +
                    "${obstacle.id}," +
                    "${obstacle.x}," +
                    "${obstacle.y}," +
                    "-1"

        return sendProtocolMessage(
            message
        )
    }


    fun updateObstacleTarget(
        obstacleId: Int,
        targetId: Int
    ): Boolean {

        return arenaGridView
            .updateObstacleTarget(
                obstacleId,
                targetId
            )
    }


    private fun sendRobotConfiguration(
        robot: ArenaGridView.Robot
    ): Boolean {

        val message =
            "ROBOT," +
                    "${robot.x}," +
                    "${robot.y}," +
                    directionCode(
                        robot.direction
                    )

        return sendProtocolMessage(
            message
        )
    }

    private fun sendRobotRemoval(
        robot: ArenaGridView.Robot
    ): Boolean {

        return sendProtocolMessage(
            "ROBOT,${robot.x},${robot.y},-1"
        )
    }

    // =====================================================
// BLUETOOTH STATUS
// =====================================================

    override fun onConnectionStateChanged(
        state: BluetoothConnectionService.ConnectionState,
        device: BluetoothDevice?
    ) {

        /*
         * Never carry an incomplete protocol message
         * across Bluetooth connections.
         */
        protocolReceiveBuffer.clear()


        when (state) {

            BluetoothConnectionService.ConnectionState.CONNECTING -> {

                vesselStatus.text =
                    "● Connecting"


                vesselStatus.setTextColor(
                    ContextCompat.getColor(
                        requireContext(),
                        R.color.odyssey_bronze_light
                    )
                )
            }


            BluetoothConnectionService.ConnectionState.CONNECTED -> {

                updateBluetoothStatus(
                    true
                )
            }


            BluetoothConnectionService.ConnectionState.DISCONNECTED,
            BluetoothConnectionService.ConnectionState.FAILED -> {

                updateBluetoothStatus(
                    false
                )
            }
        }
    }


    override fun onMessageReceived(
        message: String
    ) {

        protocolReceiveBuffer.append(
            message
        )


        while (true) {

            // =================================================
            // FIND START OF MESSAGE
            // =================================================

            val startIndex =
                protocolReceiveBuffer.indexOf(
                    "//"
                )


            if (startIndex == -1) {

                /*
                 * Preserve one trailing "/" in case the
                 * Bluetooth stream split "//" across reads.
                 *
                 * Example:
                 * read 1 = "/"
                 * read 2 = "/ROBOT,2,3,N//"
                 */
                if (
                    protocolReceiveBuffer.endsWith(
                        "/"
                    )
                ) {

                    val lastSlash =
                        protocolReceiveBuffer
                            .last()
                            .toString()


                    protocolReceiveBuffer.clear()

                    protocolReceiveBuffer.append(
                        lastSlash
                    )

                } else {

                    protocolReceiveBuffer.clear()
                }


                return
            }


            // Remove anything before the opening //
            if (startIndex > 0) {

                protocolReceiveBuffer.delete(
                    0,
                    startIndex
                )
            }


            // =================================================
            // FIND END OF MESSAGE
            // =================================================

            val endIndex =
                protocolReceiveBuffer.indexOf(
                    "//",
                    startIndex = 2
                )


            if (endIndex == -1) {

                /*
                 * Message has started but has not finished yet.
                 * Keep it in the buffer and wait for more bytes.
                 */
                return
            }


            // =================================================
            // EXTRACT PAYLOAD
            // =================================================

            val payload =
                protocolReceiveBuffer
                    .substring(
                        2,
                        endIndex
                    )
                    .trim()


            // Remove:
            //
            // //payload//
            //
            // from the buffer.
            protocolReceiveBuffer.delete(
                0,
                endIndex + 2
            )


            if (payload.isNotEmpty()) {

                handleProtocolMessage(
                    payload
                )
            }


            /*
             * Loop again because Bluetooth may have delivered:
             *
             * //ROBOT,...////TARGET,...//
             *
             * in one read.
             */
        }
    }



    private fun handleProtocolMessage(
        message: String
    ) {

        // Accept both:
        // TARGET,3,11
        // /TARGET,3,11
        val cleanMessage =
            message.removePrefix("/")

        // =================================================
        // C.4
        // STATUS,<message>
        // =================================================

        if (
            cleanMessage.startsWith(
                "STATUS,",
                ignoreCase = true
            )
        ) {

            val statusText =
                cleanMessage
                    .substringAfter(",")
                    .trim()


            if (statusText.isNotEmpty()) {

                vesselTaskStatus.text =
                    statusText
            }


            return
        }


        val parts =
            cleanMessage
                .split(",")
                .map {
                    it.trim()
                }


        when (
            parts
                .firstOrNull()
                ?.uppercase()
        ) {

            // =================================================
            // C.9
            // TARGET,<Obstacle Number>,<Target ID>
            // =================================================

            "TARGET" -> {

                if (parts.size != 3) {
                    return
                }


                val obstacleId =
                    parts[1].toIntOrNull()
                        ?: return


                val targetId =
                    parts[2].toIntOrNull()
                        ?: return


                updateObstacleTarget(
                    obstacleId,
                    targetId
                )
            }


            // =================================================
            // C.10
            // ROBOT,<x>,<y>,<direction>
            // =================================================

            "ROBOT" -> {

                if (parts.size != 4) {
                    return
                }


                val x =
                    parts[1].toIntOrNull()
                        ?: return


                val y =
                    parts[2].toIntOrNull()
                        ?: return


                val direction =
                    parseDirectionCode(
                        parts[3]
                    )
                        ?: return


                updateRobotFromExternal(
                    x,
                    y,
                    direction
                )
            }
        }
    }

    private fun parseDirectionCode(
        value: String
    ): ArenaGridView.Direction? {

        return when (
            value.uppercase()
        ) {

            "N" ->
                ArenaGridView.Direction.NORTH

            "E" ->
                ArenaGridView.Direction.EAST

            "S" ->
                ArenaGridView.Direction.SOUTH

            "W" ->
                ArenaGridView.Direction.WEST

            else ->
                null
        }
    }


    private fun updateBluetoothStatus(
        connected: Boolean
    ) {

        if (connected) {

            vesselStatus.text =
                "Connected"

            vesselStatus.setTextColor(
                Color.rgb(
                    46,
                    125,
                    50
                )
            )


            vesselTaskStatus.text =
                "--"

        } else {

            vesselStatus.text =
                "Offline"

            vesselStatus.setTextColor(
                ContextCompat.getColor(
                    requireContext(),
                    R.color.odyssey_text_muted
                )
            )


            vesselTaskStatus.text =
                "--"
        }
    }

    override fun onDestroyView() {

        BluetoothSession.removeListener(
            this
        )

        super.onDestroyView()
    }

    fun sendFullMap(): Boolean {

        if (!BluetoothSession.isConnected || arenaGridView.getRobot() == null ||
            arenaGridView.getObstacles().isEmpty()) {
            return false
        }


        val message =
            StringBuilder(
                "MAP"
            )


        // -----------------------------------------------------
        // ROBOT
        // -----------------------------------------------------

        arenaGridView
            .getRobot()
            ?.let { robot ->

                message.append(
                    "/ROBOT," +
                            "${robot.x}," +
                            "${robot.y}," +
                            directionCode(
                                robot.direction
                            )
                )
            }


        // -----------------------------------------------------
        // OBSTACLES
        // -----------------------------------------------------

        arenaGridView
            .getObstacles()
            .sortedBy {
                it.id
            }
            .forEach { obstacle ->

                message.append(
                    "/OBSTACLE," +
                            "${obstacle.id}," +
                            "${obstacle.x}," +
                            "${obstacle.y}," +
                            directionCode(
                                obstacle.direction
                            )
                )
            }


        val payload =
            message.toString()


        return BluetoothSession.sendMap(payload)
    }


    companion object {

        private const val SAVED_LAYOUT_PREFERENCES =
            "saved_layout_preferences"

        private const val SAVED_LAYOUT_KEY =
            "saved_layout"
    }
}
