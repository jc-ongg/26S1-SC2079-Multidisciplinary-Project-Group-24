package com.example.mdpgroup24.views

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import com.example.mdpgroup24.R
import kotlin.math.floor
import kotlin.math.min
import android.view.ViewConfiguration
import kotlin.math.hypot
import android.graphics.Bitmap
import android.graphics.BitmapFactory

class ArenaGridView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private val gridSize = 20


    enum class SelectionMode {
        NONE,
        OBSTACLE,
        ROBOT
    }


    enum class Direction {
        NORTH,
        EAST,
        SOUTH,
        WEST
    }


    data class GridCell(
        val x: Int,
        val y: Int
    )


    data class Obstacle(
        val id: Int,
        val x: Int,
        val y: Int,
        val direction: Direction,
        val targetId: Int? = null
    )


    data class Robot(
        val x: Int,
        val y: Int,
        val direction: Direction
    )

    private sealed class PlacementEntry {

        data class ObstacleEntry(
            val obstacleId: Int
        ) : PlacementEntry()

        data object RobotEntry :
            PlacementEntry()
    }

    sealed class UndoResult {

        data class ObstacleRemoved(
            val obstacle: Obstacle
        ) : UndoResult()


        data class RobotRemoved(
            val robot: Robot
        ) : UndoResult()
    }


    interface SelectionListener {

        fun onGridCellSelected(
            mode: SelectionMode,
            x: Int,
            y: Int
        )

        fun onInvalidGridSelection(
            mode: SelectionMode,
            x: Int,
            y: Int
        ) {
        }
    }

    interface ObstacleInteractionListener {

        fun onObstacleTapped(
            obstacle: Obstacle
        )


        fun onObstacleMoved(
            obstacle: Obstacle
        ) {
        }


        fun onObstacleRemoved(
            obstacle: Obstacle
        ) {
        }
    }

    private var selectionMode =
        SelectionMode.NONE

    private var selectedCell:
            GridCell? = null

    private var selectionListener:
            SelectionListener? = null


    private val obstacles =
        mutableListOf<Obstacle>()

    private var nextObstacleId =
        1

    private val placementHistory =
        mutableListOf<PlacementEntry>()

    private var robot:
            Robot? = null

    private var obstacleInteractionListener:
            ObstacleInteractionListener? = null


    private var pressedObstacleId:
            Int? = null

    private val obstacleCornerIdPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {

            color =
                Color.WHITE

            textSize =
                spToPx(6f)

            textAlign =
                Paint.Align.RIGHT

            typeface =
                android.graphics.Typeface.DEFAULT_BOLD
        }


    private val robotBoatBitmap: Bitmap =
        BitmapFactory.decodeResource(
            resources,
            R.drawable.robot_boat
        )


    // =====================================================
    // PAINT
    // =====================================================

    private val gridPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(R.color.grid_line, null)
            style = Paint.Style.STROKE
            strokeWidth = 1f
        }


    private val borderPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(R.color.grid_border, null)
            style = Paint.Style.STROKE
            strokeWidth = 2f
        }


    private val obstacleFacingBorderPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {

            color = resources.getColor(
                R.color.odyssey_navy,
                null
            )

            style = Paint.Style.STROKE
            strokeWidth = dpToPx(2f)
            strokeCap = Paint.Cap.SQUARE
        }


    private val robotFacingBorderPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {

            color = resources.getColor(
                R.color.odyssey_bronze_light,
                null
            )

            style = Paint.Style.STROKE
            strokeWidth = dpToPx(2f)
            strokeCap = Paint.Cap.SQUARE
        }

    private val robotBoatPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            isFilterBitmap = true
        }

    private val backgroundPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(R.color.grid_background, null)
            style = Paint.Style.FILL
        }


    private val selectionPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(
                R.color.odyssey_bronze_light,
                null
            )
            alpha = 140
            style = Paint.Style.FILL
        }


    private val selectionBorderPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(
                R.color.odyssey_navy,
                null
            )
            style = Paint.Style.STROKE
            strokeWidth = dpToPx(2f)
        }


    private val obstaclePaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(
                R.color.odyssey_bronze_light,
                null
            )
            style = Paint.Style.FILL
        }

    private val obstacleNumberPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {

            color =
                Color.WHITE

            textSize =
                spToPx(8f)

            textAlign =
                Paint.Align.CENTER

            typeface =
                android.graphics.Typeface.DEFAULT_BOLD
        }

    private val targetIdPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {

            color = resources.getColor(
                R.color.odyssey_navy,
                null
            )

            textSize =
                spToPx(5.5f)

            textAlign =
                Paint.Align.RIGHT

            typeface =
                android.graphics.Typeface.DEFAULT_BOLD
        }


    private val robotPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(
                R.color.odyssey_sea,
                null
            )
            style = Paint.Style.FILL
        }


    private val objectBorderPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = resources.getColor(
                R.color.odyssey_navy,
                null
            )
            style = Paint.Style.STROKE
            strokeWidth = dpToPx(2f)
        }


    private val horizontalLabelPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(65, 105, 145)
            textSize = spToPx(8f)
            textAlign = Paint.Align.CENTER
        }


    private val verticalLabelPaint =
        Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(155, 110, 60)
            textSize = spToPx(8f)
            textAlign = Paint.Align.CENTER
        }


    // =====================================================
    // SELECTION
    // =====================================================

    fun setSelectionListener(
        listener: SelectionListener?
    ) {
        selectionListener = listener
    }


    fun setSelectionMode(
        mode: SelectionMode
    ) {
        selectionMode = mode
        selectedCell = null
        invalidate()
    }


    fun clearSelection() {
        selectedCell = null
        invalidate()
    }

    fun setObstacleInteractionListener(
        listener: ObstacleInteractionListener?
    ) {

        obstacleInteractionListener =
            listener
    }

    private val touchSlop =
        ViewConfiguration
            .get(context)
            .scaledTouchSlop
            .toFloat()


    private var dragStartX =
        0f

    private var dragStartY =
        0f


    private var draggingObstacle =
        false


    private var dragTargetCell:
            GridCell? = null


    // =====================================================
    // OBSTACLE
    // =====================================================

    fun addObstacle(
        x: Int,
        y: Int,
        direction: Direction
    ): Boolean {

        if (!canPlaceObstacle(x, y)) {
            return false
        }


        val obstacle =
            Obstacle(
                id = nextObstacleId,
                x = x,
                y = y,
                direction = direction
            )


        obstacles.add(
            obstacle
        )


        placementHistory.add(
            PlacementEntry.ObstacleEntry(
                obstacleId = obstacle.id
            )
        )


        nextObstacleId++


        selectedCell =
            null


        invalidate()

        return true
    }


    private fun canPlaceObstacle(
        x: Int,
        y: Int
    ): Boolean {

        if (!isInsideGrid(x, y)) {
            return false
        }


        if (
            obstacles.any {
                it.x == x &&
                        it.y == y
            }
        ) {
            return false
        }


        if (isRobotCell(x, y)) {
            return false
        }


        return true
    }

    fun getObstacleAt(
        x: Int,
        y: Int
    ): Obstacle? {

        return obstacles.firstOrNull {
            it.x == x &&
                    it.y == y
        }
    }

    fun getObstacleById(
        obstacleId: Int
    ): Obstacle? {

        return obstacles.firstOrNull {
            it.id == obstacleId
        }
    }

    fun getObstacles():
            List<Obstacle> {

        return obstacles.toList()
    }

    fun removeObstacle(
        obstacleId: Int
    ): Boolean {

        val removed =
            obstacles.removeAll {
                it.id == obstacleId
            }


        if (!removed) {
            return false
        }


        // It no longer exists, so remove it from
        // the Undo placement stack as well.
        placementHistory.removeAll {
            it is PlacementEntry.ObstacleEntry &&
                    it.obstacleId == obstacleId
        }


        selectedCell =
            null


        invalidate()

        return true
    }

    fun updateObstacleDirection(
        obstacleId: Int,
        direction: Direction
    ): Boolean {

        val index =
            obstacles.indexOfFirst {
                it.id == obstacleId
            }


        if (index == -1) {
            return false
        }


        obstacles[index] =
            obstacles[index].copy(
                direction = direction
            )


        invalidate()

        return true
    }

    fun updateObstacleTarget(
        obstacleId: Int,
        targetId: Int
    ): Boolean {

        val index =
            obstacles.indexOfFirst {
                it.id == obstacleId
            }

        if (index == -1) {
            return false
        }

        obstacles[index] =
            obstacles[index].copy(
                targetId = targetId
            )

        invalidate()

        return true
    }

    private fun canMoveObstacleTo(
        obstacleId: Int,
        x: Int,
        y: Int
    ): Boolean {

        if (!isInsideGrid(x, y)) {
            return false
        }


        // Ignore the obstacle that is currently
        // being moved.
        if (
            obstacles.any {
                it.id != obstacleId &&
                        it.x == x &&
                        it.y == y
            }
        ) {
            return false
        }


        // Cannot move into the robot's 3 × 3 area.
        if (isRobotCell(x, y)) {
            return false
        }


        return true
    }


    fun moveObstacle(
        obstacleId: Int,
        x: Int,
        y: Int
    ): Boolean {

        val index =
            obstacles.indexOfFirst {
                it.id == obstacleId
            }


        if (index == -1) {
            return false
        }


        if (
            !canMoveObstacleTo(
                obstacleId,
                x,
                y
            )
        ) {
            return false
        }


        obstacles[index] =
            obstacles[index].copy(
                x = x,
                y = y
            )


        invalidate()

        return true
    }


    // =====================================================
    // ROBOT
    // =====================================================

    fun setRobot(
        x: Int,
        y: Int,
        direction: Direction
    ): Boolean {

        if (!canPlaceRobot(x, y)) {
            return false
        }


        // Robot only exists once.
        // If it is repositioned, its old history entry
        // should no longer remain in the stack.
        placementHistory.removeAll {
            it is PlacementEntry.RobotEntry
        }


        robot =
            Robot(
                x,
                y,
                direction
            )


        placementHistory.add(
            PlacementEntry.RobotEntry
        )


        selectedCell =
            null


        invalidate()

        return true
    }


    fun getRobot():
            Robot? {

        return robot
    }


    fun replaceLayout(
        savedRobot: Robot?,
        savedObstacles: List<Obstacle>
    ): Boolean {

        if (
            savedObstacles.any {
                it.id < 1 ||
                        !isInsideGrid(
                            it.x,
                            it.y
                        )
            }
        ) {
            return false
        }


        if (
            savedObstacles
                .map {
                    it.id
                }
                .distinct()
                .size != savedObstacles.size
        ) {
            return false
        }


        if (
            savedObstacles
                .map {
                    it.x to it.y
                }
                .distinct()
                .size != savedObstacles.size
        ) {
            return false
        }


        if (savedRobot != null) {

            if (
                savedRobot.x !in 1..18 ||
                savedRobot.y !in 1..18
            ) {
                return false
            }


            if (
                savedObstacles.any {
                    it.x in
                            (savedRobot.x - 1)..
                            (savedRobot.x + 1) &&
                            it.y in
                            (savedRobot.y - 1)..
                            (savedRobot.y + 1)
                }
            ) {
                return false
            }
        }


        obstacles.clear()

        obstacles.addAll(
            savedObstacles
                .sortedBy {
                    it.id
                }
        )


        robot =
            savedRobot


        placementHistory.clear()

        nextObstacleId =
            (
                    savedObstacles
                        .maxOfOrNull {
                            it.id
                        }
                        ?: 0
                    ) + 1


        selectedCell =
            null

        selectionMode =
            SelectionMode.NONE


        clearObstacleDragState()

        invalidate()

        return true
    }


    fun updateRobotFromExternal(
        x: Int,
        y: Int,
        direction: Direction
    ): Boolean {

        if (!canPlaceRobot(x, y)) {
            return false
        }


        robot =
            Robot(
                x = x,
                y = y,
                direction = direction
            )


        selectedCell =
            null


        invalidate()

        return true
    }

    // =====================================================
// ROBOT MOVEMENT
// =====================================================

    fun moveRobotForward(): Boolean {

        val currentRobot =
            robot ?: return false


        val nextX: Int
        val nextY: Int


        when (currentRobot.direction) {

            Direction.NORTH -> {
                nextX = currentRobot.x
                nextY = currentRobot.y + 1
            }

            Direction.SOUTH -> {
                nextX = currentRobot.x
                nextY = currentRobot.y - 1
            }

            Direction.EAST -> {
                nextX = currentRobot.x + 1
                nextY = currentRobot.y
            }

            Direction.WEST -> {
                nextX = currentRobot.x - 1
                nextY = currentRobot.y
            }
        }


        return moveRobotTo(
            nextX,
            nextY
        )
    }


    fun moveRobotBackward(): Boolean {

        val currentRobot =
            robot ?: return false


        val nextX: Int
        val nextY: Int


        when (currentRobot.direction) {

            Direction.NORTH -> {
                nextX = currentRobot.x
                nextY = currentRobot.y - 1
            }

            Direction.SOUTH -> {
                nextX = currentRobot.x
                nextY = currentRobot.y + 1
            }

            Direction.EAST -> {
                nextX = currentRobot.x - 1
                nextY = currentRobot.y
            }

            Direction.WEST -> {
                nextX = currentRobot.x + 1
                nextY = currentRobot.y
            }
        }


        return moveRobotTo(
            nextX,
            nextY
        )
    }


    private fun moveRobotTo(
        x: Int,
        y: Int
    ): Boolean {

        val currentRobot =
            robot ?: return false


        if (!canPlaceRobot(x, y)) {
            return false
        }


        robot =
            currentRobot.copy(
                x = x,
                y = y
            )


        invalidate()

        return true
    }


    fun rotateRobotLeft(): Boolean {

        val currentRobot =
            robot ?: return false


        val newDirection =
            when (currentRobot.direction) {

                Direction.NORTH ->
                    Direction.WEST

                Direction.WEST ->
                    Direction.SOUTH

                Direction.SOUTH ->
                    Direction.EAST

                Direction.EAST ->
                    Direction.NORTH
            }


        robot =
            currentRobot.copy(
                direction = newDirection
            )


        invalidate()

        return true
    }


    fun rotateRobotRight(): Boolean {

        val currentRobot =
            robot ?: return false


        val newDirection =
            when (currentRobot.direction) {

                Direction.NORTH ->
                    Direction.EAST

                Direction.EAST ->
                    Direction.SOUTH

                Direction.SOUTH ->
                    Direction.WEST

                Direction.WEST ->
                    Direction.NORTH
            }


        robot =
            currentRobot.copy(
                direction = newDirection
            )


        invalidate()

        return true
    }


    private fun canPlaceRobot(
        centreX: Int,
        centreY: Int
    ): Boolean {

        // Centre must leave one cell around
        // every side for the 3x3 robot.
        if (
            centreX !in 1..18 ||
            centreY !in 1..18
        ) {
            return false
        }


        for (
        x in
        (centreX - 1)..(centreX + 1)
        ) {

            for (
            y in
            (centreY - 1)..(centreY + 1)
            ) {

                if (
                    obstacles.any {
                        it.x == x &&
                                it.y == y
                    }
                ) {
                    return false
                }
            }
        }


        return true
    }


    private fun isRobotCell(
        x: Int,
        y: Int
    ): Boolean {

        val currentRobot =
            robot ?: return false


        return (
                x in
                        (currentRobot.x - 1)..
                        (currentRobot.x + 1)
                        &&
                        y in
                        (currentRobot.y - 1)..
                        (currentRobot.y + 1)
                )
    }


    // =====================================================
    // TOUCH
    // =====================================================

    override fun onTouchEvent(
        event: MotionEvent
    ): Boolean {


        // =====================================================
        // NORMAL MODE
        //
        // Existing obstacles may be tapped or dragged.
        // =====================================================

        if (
            selectionMode ==
            SelectionMode.NONE
        ) {

            when (event.actionMasked) {

                // -------------------------------------------------
                // TOUCH START
                // -------------------------------------------------

                MotionEvent.ACTION_DOWN -> {

                    val cell =
                        coordinatesToGridCell(
                            event.x,
                            event.y
                        )


                    val obstacle =
                        if (cell != null) {

                            getObstacleAt(
                                cell.x,
                                cell.y
                            )

                        } else {

                            null
                        }


                    // Empty cell:
                    // grid has nothing to interact with.
                    if (obstacle == null) {

                        clearObstacleDragState()

                        return false
                    }


                    pressedObstacleId =
                        obstacle.id


                    dragStartX =
                        event.x

                    dragStartY =
                        event.y


                    draggingObstacle =
                        false


                    dragTargetCell =
                        GridCell(
                            obstacle.x,
                            obstacle.y
                        )


                    parent
                        ?.requestDisallowInterceptTouchEvent(
                            true
                        )


                    return true
                }


                // -------------------------------------------------
                // DRAG
                // -------------------------------------------------

                MotionEvent.ACTION_MOVE -> {

                    val obstacleId =
                        pressedObstacleId
                            ?: return false


                    val distance =
                        hypot(
                            event.x - dragStartX,
                            event.y - dragStartY
                        )


                    // Do not interpret tiny finger movements
                    // as dragging.
                    if (
                        !draggingObstacle &&
                        distance > touchSlop
                    ) {

                        draggingObstacle =
                            true
                    }


                    if (draggingObstacle) {

                        val cell =
                            coordinatesToGridCell(
                                event.x,
                                event.y
                            )


                        dragTargetCell =
                            if (
                                cell != null &&
                                canMoveObstacleTo(
                                    obstacleId,
                                    cell.x,
                                    cell.y
                                )
                            ) {

                                cell

                            } else {

                                null
                            }


                        invalidate()
                    }


                    return true
                }


                // -------------------------------------------------
                // RELEASE
                // -------------------------------------------------

                MotionEvent.ACTION_UP -> {

                    val obstacleId =
                        pressedObstacleId


                    if (obstacleId == null) {

                        releaseParentTouch()

                        return false
                    }


                    if (draggingObstacle) {

                        val cell =
                            coordinatesToGridCell(
                                event.x,
                                event.y
                            )


                        if (cell == null) {

                            // Keep the old data because once removed,
                            // it will no longer exist in the list.
                            val removedObstacle =
                                getObstacleById(
                                    obstacleId
                                )


                            if (
                                removedObstacle != null &&
                                removeObstacle(
                                    obstacleId
                                )
                            ) {

                                obstacleInteractionListener
                                    ?.onObstacleRemoved(
                                        removedObstacle
                                    )
                            }

                        } else if (
                            canMoveObstacleTo(
                                obstacleId,
                                cell.x,
                                cell.y
                            )
                        ) {

                            val success =
                                moveObstacle(
                                    obstacleId,
                                    cell.x,
                                    cell.y
                                )


                            if (success) {

                                getObstacleById(
                                    obstacleId
                                )?.let { movedObstacle ->

                                    obstacleInteractionListener
                                        ?.onObstacleMoved(
                                            movedObstacle
                                        )
                                }
                            }
                        }
                        else {

                            // =====================================
                            // INVALID DROP
                            //
                            // Do nothing.
                            // Since we did not change the real
                            // obstacle during the drag, it simply
                            // remains at its original position.
                            // =====================================
                        }

                    } else {

                        // =========================================
                        // NORMAL TAP
                        // =========================================

                        val cell =
                            coordinatesToGridCell(
                                event.x,
                                event.y
                            )


                        val releasedObstacle =
                            if (cell != null) {

                                getObstacleAt(
                                    cell.x,
                                    cell.y
                                )

                            } else {

                                null
                            }


                        if (
                            releasedObstacle != null &&
                            releasedObstacle.id ==
                            obstacleId
                        ) {

                            performClick()


                            obstacleInteractionListener
                                ?.onObstacleTapped(
                                    releasedObstacle
                                )
                        }
                    }


                    clearObstacleDragState()

                    releaseParentTouch()

                    invalidate()


                    return true
                }


                // -------------------------------------------------
                // CANCEL
                // -------------------------------------------------

                MotionEvent.ACTION_CANCEL -> {

                    clearObstacleDragState()

                    releaseParentTouch()

                    invalidate()


                    return true
                }
            }


            return false
        }


        // =====================================================
        // PLACEMENT MODE
        //
        // Keep your existing obstacle / robot placement logic.
        // =====================================================

        when (event.actionMasked) {

            MotionEvent.ACTION_DOWN -> {

                parent
                    ?.requestDisallowInterceptTouchEvent(
                        true
                    )


                updateSelection(
                    event.x,
                    event.y
                )


                return true
            }


            MotionEvent.ACTION_MOVE -> {

                updateSelection(
                    event.x,
                    event.y
                )


                return true
            }


            MotionEvent.ACTION_UP -> {

                performClick()


                val cell =
                    coordinatesToGridCell(
                        event.x,
                        event.y
                    )


                if (
                    cell != null &&
                    isValidSelection(
                        cell.x,
                        cell.y
                    )
                ) {

                    selectedCell =
                        cell


                    invalidate()


                    selectionListener
                        ?.onGridCellSelected(
                            selectionMode,
                            cell.x,
                            cell.y
                        )

                } else {

                    if (cell != null) {

                        selectionListener
                            ?.onInvalidGridSelection(
                                selectionMode,
                                cell.x,
                                cell.y
                            )
                    }


                    selectedCell =
                        null


                    invalidate()
                }


                parent
                    ?.requestDisallowInterceptTouchEvent(
                        false
                    )


                return true
            }


            MotionEvent.ACTION_CANCEL -> {

                selectedCell =
                    null


                invalidate()


                parent
                    ?.requestDisallowInterceptTouchEvent(
                        false
                    )


                return true
            }
        }


        return true
    }

    private fun clearObstacleDragState() {

        pressedObstacleId =
            null

        draggingObstacle =
            false

        dragTargetCell =
            null
    }

    private fun releaseParentTouch() {

        parent
            ?.requestDisallowInterceptTouchEvent(
                false
            )
    }

    private fun drawObstacleDragPreview(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        if (!draggingObstacle) {
            return
        }


        val obstacleId =
            pressedObstacleId
                ?: return


        val cell =
            dragTargetCell
                ?: return


        val obstacle =
            getObstacleById(
                obstacleId
            )
                ?: return


        val rect =
            cellToRect(
                geometry,
                cell.x,
                cell.y
            )


        canvas.drawRect(
            rect,
            selectionPaint
        )


        canvas.drawRect(
            rect,
            selectionBorderPaint
        )


        drawFacingBorder(
            canvas,
            rect,
            obstacle.direction,
            obstacleFacingBorderPaint
        )


        val verticalOffset =
            (
                    obstacleNumberPaint.descent() +
                            obstacleNumberPaint.ascent()
                    ) / 2f


        canvas.drawText(
            obstacle.id.toString(),
            rect.centerX(),
            rect.centerY() - verticalOffset,
            obstacleNumberPaint
        )
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }


    private fun updateSelection(
        touchX: Float,
        touchY: Float
    ) {

        val cell =
            coordinatesToGridCell(
                touchX,
                touchY
            )


        selectedCell =
            if (
                cell != null &&
                isValidSelection(
                    cell.x,
                    cell.y
                )
            ) {
                cell
            } else {
                null
            }


        invalidate()
    }


    private fun isValidSelection(
        x: Int,
        y: Int
    ): Boolean {

        return when (selectionMode) {

            SelectionMode.NONE ->
                false

            SelectionMode.OBSTACLE ->
                canPlaceObstacle(
                    x,
                    y
                )

            SelectionMode.ROBOT ->
                canPlaceRobot(
                    x,
                    y
                )
        }
    }


    // =====================================================
    // COORDINATE CONVERSION
    // =====================================================

    private fun coordinatesToGridCell(
        touchX: Float,
        touchY: Float
    ): GridCell? {

        val geometry =
            calculateGridGeometry()


        if (
            touchX < geometry.left ||
            touchX >= geometry.right ||
            touchY < geometry.top ||
            touchY >= geometry.bottom
        ) {
            return null
        }


        val x =
            floor(
                (
                        touchX -
                                geometry.left
                        ) /
                        geometry.cellSize
            ).toInt()


        val rowFromTop =
            floor(
                (
                        touchY -
                                geometry.top
                        ) /
                        geometry.cellSize
            ).toInt()


        val y =
            gridSize -
                    1 -
                    rowFromTop


        return if (
            isInsideGrid(
                x,
                y
            )
        ) {
            GridCell(x, y)
        } else {
            null
        }
    }


    private fun isInsideGrid(
        x: Int,
        y: Int
    ): Boolean {

        return (
                x in 0 until gridSize &&
                        y in 0 until gridSize
                )
    }


    // =====================================================
    // DRAW
    // =====================================================

    override fun onDraw(
        canvas: Canvas
    ) {

        super.onDraw(canvas)


        val geometry =
            calculateGridGeometry()


        canvas.drawRect(
            RectF(
                geometry.left,
                geometry.top,
                geometry.right,
                geometry.bottom
            ),
            backgroundPaint
        )



        canvas.drawRect(
            RectF(
                geometry.left,
                geometry.top,
                geometry.right,
                geometry.bottom
            ),
            borderPaint
        )



        drawRobot(
            canvas,
            geometry
        )
        drawGridLines(
            canvas,
            geometry
        )
        drawObstacles(
            canvas,
            geometry
        )

        drawObstacleNumbers(
            canvas,
            geometry
        )

        drawFacingBorders(
            canvas,
            geometry
        )
        drawTemporarySelection(
            canvas,
            geometry
        )

        drawObstacleDragPreview(
            canvas,
            geometry
        )


        drawCoordinates(
            canvas,
            geometry
        )


    }


    private fun drawObstacles(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        obstacles.forEach {

            canvas.drawRect(
                cellToRect(
                    geometry,
                    it.x,
                    it.y
                ),
                obstaclePaint
            )
        }
    }

    private fun drawObstacleNumbers(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        obstacles.forEach { obstacle ->

            val rect =
                cellToRect(
                    geometry,
                    obstacle.x,
                    obstacle.y
                )


            // =================================================
            // OBSTACLE ID
            //
            // Main number.
            // Slightly left of the true centre.
            // =================================================

            val obstacleVerticalOffset =
                (
                        obstacleNumberPaint.descent() +
                                obstacleNumberPaint.ascent()
                        ) / 2f


            val obstacleX =
                rect.centerX() -
                        geometry.cellSize * 0.12f


            canvas.drawText(
                obstacle.id.toString(),
                obstacleX,
                rect.centerY() -
                        obstacleVerticalOffset,
                obstacleNumberPaint
            )


            // =================================================
            // TARGET ID
            //
            // Small number in the top-right.
            // Inset from both edges so the facing border
            // does not cover it.
            // =================================================

            obstacle.targetId?.let { targetId ->

                val edgeInset =
                    geometry.cellSize * 0.13f


                canvas.drawText(
                    targetId.toString(),
                    rect.right - edgeInset,
                    rect.top +
                            geometry.cellSize * 0.30f,
                    targetIdPaint
                )
            }
        }
    }

    private fun drawRobotBoat(
        canvas: Canvas,
        rect: RectF,
        direction: Direction,
        geometry: GridGeometry
    ) {

        val rotation =
            when (direction) {

                Direction.NORTH ->
                    0f

                Direction.EAST ->
                    90f

                Direction.SOUTH ->
                    180f

                Direction.WEST ->
                    270f
            }


        val padding =
            geometry.cellSize * 0.05f


        val destination =
            RectF(
                rect.left + padding,
                rect.top + padding,
                rect.right - padding,
                rect.bottom - padding
            )


        canvas.save()


        canvas.rotate(
            rotation,
            rect.centerX(),
            rect.centerY()
        )


        canvas.drawBitmap(
            robotBoatBitmap,
            null,
            destination,
            robotBoatPaint
        )


        canvas.restore()
    }

    private fun drawRobot(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        val currentRobot =
            robot ?: return


        val rect =
            robotRect(
                geometry,
                currentRobot.x,
                currentRobot.y
            )


        // 3 × 3 Odyssey sea background.
        canvas.drawRect(
            rect,
            robotPaint
        )


        // Boat artwork.
        drawRobotBoat(
            canvas,
            rect,
            currentRobot.direction,
            geometry
        )

    }


    private fun drawTemporarySelection(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        val cell =
            selectedCell ?: return


        val rect =
            when (selectionMode) {

                SelectionMode.OBSTACLE ->
                    cellToRect(
                        geometry,
                        cell.x,
                        cell.y
                    )

                SelectionMode.ROBOT ->
                    robotRect(
                        geometry,
                        cell.x,
                        cell.y
                    )

                SelectionMode.NONE ->
                    return
            }


        canvas.drawRect(
            rect,
            selectionPaint
        )


        canvas.drawRect(
            rect,
            selectionBorderPaint
        )
    }


    private fun drawGridLines(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        for (i in 0..gridSize) {

            val x =
                geometry.left +
                        i * geometry.cellSize


            canvas.drawLine(
                x,
                geometry.top,
                x,
                geometry.bottom,
                gridPaint
            )


            val y =
                geometry.top +
                        i * geometry.cellSize


            canvas.drawLine(
                geometry.left,
                y,
                geometry.right,
                y,
                gridPaint
            )
        }
    }



    private fun drawFacingBorders(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        robot?.let { currentRobot ->

            drawFacingBorder(
                canvas,
                robotRect(
                    geometry,
                    currentRobot.x,
                    currentRobot.y
                ),
                currentRobot.direction,
                robotFacingBorderPaint
            )
        }

        obstacles.forEach { obstacle ->

            drawFacingBorder(
                canvas,
                cellToRect(
                    geometry,
                    obstacle.x,
                    obstacle.y
                ),
                obstacle.direction,
                obstacleFacingBorderPaint
            )
        }
    }

    private fun drawFacingBorder(
        canvas: Canvas,
        rect: RectF,
        direction: Direction,
        paint: Paint
    ) {

        when (direction) {

            Direction.NORTH -> {

                canvas.drawLine(
                    rect.left,
                    rect.top,
                    rect.right,
                    rect.top,
                    paint
                )
            }


            Direction.EAST -> {

                canvas.drawLine(
                    rect.right,
                    rect.top,
                    rect.right,
                    rect.bottom,
                    paint
                )
            }


            Direction.SOUTH -> {

                canvas.drawLine(
                    rect.left,
                    rect.bottom,
                    rect.right,
                    rect.bottom,
                    paint
                )
            }


            Direction.WEST -> {

                canvas.drawLine(
                    rect.left,
                    rect.top,
                    rect.left,
                    rect.bottom,
                    paint
                )
            }
        }
    }


    private fun drawCoordinates(
        canvas: Canvas,
        geometry: GridGeometry
    ) {

        for (x in 0 until gridSize) {

            val centreX =
                geometry.left +
                        (x + 0.5f) *
                        geometry.cellSize


            canvas.drawText(
                x.toString(),
                centreX,
                geometry.bottom +
                        dpToPx(15f),
                horizontalLabelPaint
            )


            canvas.drawText(
                x.toString(),
                centreX,
                geometry.top -
                        dpToPx(8f),
                horizontalLabelPaint
            )
        }


        val offset =
            (
                    verticalLabelPaint.descent() +
                            verticalLabelPaint.ascent()
                    ) / 2f


        for (y in 0 until gridSize) {

            val centreY =
                geometry.bottom -
                        (y + 0.5f) *
                        geometry.cellSize


            canvas.drawText(
                y.toString(),
                geometry.left -
                        dpToPx(8f),
                centreY - offset,
                verticalLabelPaint
            )


            canvas.drawText(
                y.toString(),
                geometry.right +
                        dpToPx(8f),
                centreY - offset,
                verticalLabelPaint
            )
        }
    }


    private fun cellToRect(
        geometry: GridGeometry,
        x: Int,
        y: Int
    ): RectF {

        val left =
            geometry.left +
                    x * geometry.cellSize


        val bottom =
            geometry.bottom -
                    y * geometry.cellSize


        return RectF(
            left,
            bottom - geometry.cellSize,
            left + geometry.cellSize,
            bottom
        )
    }


    private fun robotRect(
        geometry: GridGeometry,
        centreX: Int,
        centreY: Int
    ): RectF {

        val left =
            geometry.left +
                    (centreX - 1) *
                    geometry.cellSize


        val top =
            geometry.bottom -
                    (centreY + 2) *
                    geometry.cellSize


        return RectF(
            left,
            top,
            left + 3 * geometry.cellSize,
            top + 3 * geometry.cellSize
        )
    }


    // =====================================================
    // GEOMETRY
    // =====================================================

    private data class GridGeometry(
        val left: Float,
        val top: Float,
        val right: Float,
        val bottom: Float,
        val cellSize: Float
    )


    private fun calculateGridGeometry():
            GridGeometry {

        val margin =
            dpToPx(24f)


        val availableWidth =
            width -
                    margin * 2


        val availableHeight =
            height -
                    margin * 2


        val cellSize =
            min(
                availableWidth /
                        gridSize.toFloat(),

                availableHeight /
                        gridSize.toFloat()
            )


        val gridPixels =
            cellSize *
                    gridSize


        val left =
            (
                    width -
                            gridPixels
                    ) / 2f


        val top =
            (
                    height -
                            gridPixels
                    ) / 2f


        return GridGeometry(
            left,
            top,
            left + gridPixels,
            top + gridPixels,
            cellSize
        )
    }


    private fun dpToPx(
        dp: Float
    ): Float {

        return dp *
                resources.displayMetrics.density
    }


    private fun spToPx(
        sp: Float
    ): Float {

        return sp *
                resources.displayMetrics.scaledDensity
    }

    fun undoLastPlacement():
            UndoResult? {

        if (placementHistory.isEmpty()) {
            return null
        }


        val lastEntry =
            placementHistory.removeAt(
                placementHistory.lastIndex
            )


        val result =
            when (lastEntry) {

                is PlacementEntry.ObstacleEntry -> {

                    val obstacle =
                        obstacles.firstOrNull {
                            it.id ==
                                    lastEntry.obstacleId
                        }
                            ?: return null


                    obstacles.removeAll {
                        it.id ==
                                lastEntry.obstacleId
                    }


                    // Undo should allow the latest obstacle ID
                    // to be reused.
                    if (
                        lastEntry.obstacleId ==
                        nextObstacleId - 1
                    ) {

                        nextObstacleId--
                    }


                    UndoResult.ObstacleRemoved(
                        obstacle
                    )
                }

                is PlacementEntry.RobotEntry -> {

                    val removedRobot =
                        robot ?: return null


                    robot =
                        null


                    UndoResult.RobotRemoved(
                        removedRobot
                    )
                }
            }


        selectedCell =
            null


        invalidate()


        return result
    }

    fun resetArena() {

        obstacles.clear()

        robot =
            null

        placementHistory.clear()


        nextObstacleId =
            1


        selectedCell =
            null

        selectionMode =
            SelectionMode.NONE


        clearObstacleDragState()


        invalidate()
    }
}
