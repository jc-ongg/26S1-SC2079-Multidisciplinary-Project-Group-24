package com.example.mdpgroup24

import com.example.mdpgroup24.bluetooth.Task1Preparation
import com.example.mdpgroup24.bluetooth.BluetoothFrameBuffer
import com.example.mdpgroup24.bluetooth.ChatHistory
import org.junit.Assert.*
import org.junit.Test

class Task1PreparationTest {
    @Test fun startRequiresLatestReadyReply() {
        val state = Task1Preparation()
        assertFalse(state.start())
        state.submit("first")
        assertFalse(state.start())
        state.submit("second")
        assertFalse(state.receive("PLAN,first,READY"))
        assertFalse(state.canStart)
        assertTrue(state.receive("PLAN,second,READY"))
        assertTrue(state.start())
        assertFalse(state.start())
    }

    @Test fun editsDisconnectAndResetInvalidateConfirmation() {
        val state = Task1Preparation()
        state.submit("first")
        state.receive("PLAN,first,READY")
        state.invalidate()
        assertFalse(state.canStart)
        assertFalse(state.receive("PLAN,first,READY"))
    }

    @Test fun errorDoesNotEnableStart() {
        val state = Task1Preparation()
        state.submit("first")
        state.receive("PLAN,first,ERROR,Algo unreachable")
        assertFalse(state.canStart)
        assertEquals("Algo unreachable", state.error)
    }

    @Test fun fragmentedAndBatchedFindingsAreLoggedOnceAsCompleteMessages() {
        val frames = BluetoothFrameBuffer()
        val history = ChatHistory()
        assertTrue(frames.feed("/").frames.isEmpty())
        assertTrue(frames.feed("/TARG").frames.isEmpty())
        val batch = frames.feed("ET,3,11////ROBOT,1,2,N////TARGET,4,38//")
        batch.frames.forEach(history::receiveFrame)
        assertEquals(listOf("Target found: obstacle 3, image ID 11",
            "Target found: obstacle 4, image ID 38"), history.snapshot().map { it.message })
    }

    @Test fun malformedTargetCannotCreateAFinding() {
        val history = ChatHistory()
        history.receiveFrame("TARGET,1,NA")
        history.receiveFrame("TARGET,1")
        assertTrue(history.snapshot().isEmpty())
    }

    @Test fun recreatedChatCanReplaySessionHistoryAndClearIt() {
        val history = ChatHistory()
        history.receiveFrame("TARGET,3,11")
        val firstView = history.snapshot()
        assertEquals(firstView, history.snapshot())
        history.clear()
        assertTrue(history.snapshot().isEmpty())
        assertEquals(1, firstView.size)
    }

    @Test fun reconnectClearsAPartialFrame() {
        val frames = BluetoothFrameBuffer()
        frames.feed("//TARGET,3,")
        frames.clear()
        assertTrue(frames.feed("11//").frames.isEmpty())
    }
}
