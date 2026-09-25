package com.example.mdpgroup24.fragments

import android.bluetooth.BluetoothDevice
import android.graphics.drawable.GradientDrawable
import android.os.Bundle
import android.view.Gravity
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import com.example.mdpgroup24.R
import com.example.mdpgroup24.bluetooth.BluetoothConnectionService
import com.example.mdpgroup24.bluetooth.BluetoothSession

class ChatFragment :
    Fragment(R.layout.fragment_chat),
    BluetoothSession.Listener {

    private lateinit var messageInput: EditText
    private lateinit var sendButton: Button
    private lateinit var clearButton: TextView

    private lateinit var suggestedTest: Button
    private lateinit var suggestedForward: Button
    private lateinit var suggestedBack: Button

    private lateinit var messagesContainer: LinearLayout
    private lateinit var chatScrollView: ScrollView


    // ============================================================
    // SETUP
    // ============================================================

    override fun onViewCreated(
        view: View,
        savedInstanceState: Bundle?
    ) {
        super.onViewCreated(view, savedInstanceState)

        messageInput =
            view.findViewById(R.id.messageInput)

        sendButton =
            view.findViewById(R.id.btnSend)

        clearButton =
            view.findViewById(R.id.btnClearHistory)

        suggestedTest =
            view.findViewById(R.id.btnSuggested1)

        suggestedForward =
            view.findViewById(R.id.btnSuggested2)

        suggestedBack =
            view.findViewById(R.id.btnSuggested3)

        messagesContainer =
            view.findViewById(R.id.chatMessagesContainer)

        chatScrollView =
            view.findViewById(R.id.chatScrollView)


        BluetoothSession.initialize(
            requireContext()
        )

        BluetoothSession.addListener(
            this
        )


        messagesContainer.removeAllViews()


        parentFragmentManager
            .setFragmentResultListener(
                RESULT_EXTERNAL_SENT_MESSAGE,
                viewLifecycleOwner
            ) { _, bundle ->

                bundle
                    .getString(KEY_MESSAGE)
                    ?.takeIf {
                        it.isNotBlank()
                    }
                    ?.let {
                        addSentMessage(it)
                    }
            }


        // Normal send button
        sendButton.setOnClickListener {
            sendCurrentMessage()
        }


        // Suggested commands
        suggestedTest.setOnClickListener {
            sendSuggestedMessage("Test")
        }

        suggestedForward.setOnClickListener {
            sendSuggestedMessage("Gr24")
        }

        suggestedBack.setOnClickListener {
            sendSuggestedMessage("Chat")
        }


        // Clear history
        clearButton.setOnClickListener {
            messagesContainer.removeAllViews()
            messageInput.setText("")
        }
    }


    // ============================================================
    // SEND
    // ============================================================

    private fun sendCurrentMessage() {

        val message =
            messageInput.text
                .toString()
                .trim()

        if (message.isEmpty()) {
            return
        }

        sendMessage(message)
    }


    private fun sendSuggestedMessage(
        message: String
    ) {
        messageInput.setText(message)

        sendMessage(message)

        messageInput.setText("")
    }


    private fun sendMessage(
        message: String
    ) {

        if (!BluetoothSession.isConnected) {

            addSystemMessage(
                "Bluetooth not connected"
            )

            return
        }


        val sent =
            BluetoothSession.send(message)


        if (sent) {

            addSentMessage(message)

            messageInput.setText("")

        } else {

            addSystemMessage(
                "Failed to send message"
            )
        }
    }


    // ============================================================
    // RECEIVE
    // ============================================================

    override fun onMessageReceived(
        message: String
    ) {

        if (!isAdded || view == null) {
            return
        }

        addReceivedMessage(
            message.trim()
        )
    }


    override fun onConnectionStateChanged(
        state: BluetoothConnectionService.ConnectionState,
        device: BluetoothDevice?
    ) {
        // Connection status is displayed in BluetoothFragment.
    }


    // ============================================================
    // SENT MESSAGE
    // ============================================================

    private fun addSentMessage(
        message: String
    ) {

        val bubble =
            createMessageBubble(message)


        bubble.setTextColor(
            color(R.color.odyssey_navy)
        )


        bubble.background =
            createBubbleBackground(
                color(R.color.grid_background),
                color(R.color.odyssey_navy)
            )


        val params =
            createBubbleLayoutParams().apply {
                gravity = Gravity.END
                marginStart = dpToPx(70)
            }


        bubble.layoutParams =
            params


        messagesContainer.addView(
            bubble
        )


        scrollToLatestMessage()
    }


    // ============================================================
    // RECEIVED MESSAGE
    // ============================================================

    private fun addReceivedMessage(
        message: String
    ) {

        if (message.isEmpty()) {
            return
        }


        val bubble =
            createMessageBubble(message)


        bubble.setTextColor(
            color(R.color.odyssey_text)
        )


        bubble.background =
            createBubbleBackground(
                color(R.color.odyssey_parchment),
                color(R.color.odyssey_bronze_light)
            )


        val params =
            createBubbleLayoutParams().apply {
                gravity = Gravity.START
                marginEnd = dpToPx(70)
            }


        bubble.layoutParams =
            params


        messagesContainer.addView(
            bubble
        )


        scrollToLatestMessage()
    }


    // ============================================================
    // SYSTEM MESSAGE
    // ============================================================

    private fun addSystemMessage(
        message: String
    ) {

        val textView =
            TextView(requireContext()).apply {

                text = message
                textSize = 12f
                gravity = Gravity.CENTER

                setTextColor(
                    color(
                        R.color.odyssey_text_muted
                    )
                )
            }


        textView.layoutParams =
            LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
            ).apply {

                topMargin =
                    dpToPx(7)

                bottomMargin =
                    dpToPx(7)
            }


        messagesContainer.addView(
            textView
        )


        scrollToLatestMessage()
    }


    // ============================================================
    // BUBBLE DESIGN
    // ============================================================

    private fun createMessageBubble(
        message: String
    ): TextView {

        return TextView(requireContext()).apply {

            text = message

            textSize = 15f

            maxWidth =
                (
                        resources.displayMetrics.widthPixels *
                                0.65f
                        ).toInt()

            setPadding(
                dpToPx(14),
                dpToPx(10),
                dpToPx(14),
                dpToPx(10)
            )
        }
    }


    private fun createBubbleBackground(
        backgroundColor: Int,
        borderColor: Int? = null
    ): GradientDrawable {

        return GradientDrawable().apply {

            shape =
                GradientDrawable.RECTANGLE

            setColor(
                backgroundColor
            )

            cornerRadius =
                dpToPx(16).toFloat()


            if (borderColor != null) {

                setStroke(
                    dpToPx(1),
                    borderColor
                )
            }
        }
    }


    private fun createBubbleLayoutParams():
            LinearLayout.LayoutParams {

        return LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.WRAP_CONTENT,
            LinearLayout.LayoutParams.WRAP_CONTENT
        ).apply {

            topMargin =
                dpToPx(5)

            bottomMargin =
                dpToPx(5)
        }
    }


    // ============================================================
    // AUTO SCROLL
    // ============================================================

    private fun scrollToLatestMessage() {

        chatScrollView.post {

            chatScrollView.fullScroll(
                View.FOCUS_DOWN
            )
        }
    }


    // ============================================================
    // HELPERS
    // ============================================================

    private fun color(
        resource: Int
    ): Int {

        return ContextCompat.getColor(
            requireContext(),
            resource
        )
    }


    private fun dpToPx(
        dp: Int
    ): Int {

        return (
                dp *
                        resources.displayMetrics.density
                ).toInt()
    }


    // ============================================================
    // CLEANUP
    // ============================================================

    override fun onDestroyView() {

        BluetoothSession.removeListener(
            this
        )

        super.onDestroyView()
    }


    companion object {

        const val RESULT_EXTERNAL_SENT_MESSAGE =
            "chat_external_sent_message"

        const val KEY_MESSAGE =
            "message"
    }
}
