package com.example.mdpgroup24

import android.os.Bundle
import android.view.MotionEvent
import android.view.View
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.viewpager2.widget.ViewPager2
import com.example.mdpgroup24.adapters.MainPagerAdapter
import androidx.core.content.ContextCompat
import android.graphics.Typeface
import kotlin.math.abs

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        enableEdgeToEdge()
        setContentView(R.layout.activity_main)

        // Keep app content above Android system bars
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->

            val systemBars =
                insets.getInsets(WindowInsetsCompat.Type.systemBars())

            v.setPadding(
                systemBars.left,
                systemBars.top,
                systemBars.right,
                systemBars.bottom
            )

            insets
        }


        // =====================================================
        // MAIN TABS
        // =====================================================

        val tabHome =
            findViewById<TextView>(R.id.tabHome)

        val tabBluetooth =
            findViewById<TextView>(R.id.tabBluetooth)

        val mainTabBar =
            findViewById<LinearLayout>(R.id.mainTabBar)

        val viewPager =
            findViewById<ViewPager2>(R.id.mainViewPager)


        // Connect HOME and BLUETOOTH pages
        viewPager.adapter = MainPagerAdapter(this)

        // Disable swiping on the actual page content
        viewPager.isUserInputEnabled = false


        // =====================================================
        // TAB TAPS
        // =====================================================

        tabHome.setOnClickListener {
            viewPager.currentItem = 0
        }

        tabBluetooth.setOnClickListener {
            viewPager.currentItem = 1
        }


        // =====================================================
        // SWIPE ONLY ON MAIN TAB BAR
        // =====================================================

        var startX = 0f

        val swipeListener = View.OnTouchListener { touchedView, event ->

            when (event.actionMasked) {

                MotionEvent.ACTION_DOWN -> {
                    startX = event.x
                }


                MotionEvent.ACTION_UP -> {

                    val differenceX = event.x - startX

                    if (abs(differenceX) > 80) {

                        if (differenceX < 0) {

                            // Swipe left:
                            // HOME -> BLUETOOTH
                            if (viewPager.currentItem < 1) {
                                viewPager.currentItem += 1
                            }

                        } else {

                            // Swipe right:
                            // BLUETOOTH -> HOME
                            if (viewPager.currentItem > 0) {
                                viewPager.currentItem -= 1
                            }
                        }

                    } else {

                        // Small movement = normal tap
                        touchedView.performClick()
                    }
                }
            }

            true
        }


        // Swipe works anywhere on the tab bar,
        // including directly on HOME or BLUETOOTH.
        mainTabBar.setOnTouchListener(swipeListener)

        tabHome.setOnTouchListener(swipeListener)

        tabBluetooth.setOnTouchListener(swipeListener)


        // =====================================================
        // UPDATE SELECTED TAB APPEARANCE
        // =====================================================

        viewPager.registerOnPageChangeCallback(
            object : ViewPager2.OnPageChangeCallback() {

                override fun onPageSelected(position: Int) {
                    super.onPageSelected(position)

                    updateTabAppearance(
                        position,
                        tabHome,
                        tabBluetooth
                    )
                }
            }
        )


        // HOME selected by default
        updateTabAppearance(
            0,
            tabHome,
            tabBluetooth
        )
    }


    private fun updateTabAppearance(
        selectedPage: Int,
        tabHome: TextView,
        tabBluetooth: TextView
    ) {

        val selectedTextColor =
            ContextCompat.getColor(this, R.color.odyssey_cream)

        val unselectedTextColor =
            ContextCompat.getColor(this, R.color.odyssey_navy)

        // Reset both
        tabHome.setBackgroundResource(R.drawable.bg_main_tab_unselected)
        tabBluetooth.setBackgroundResource(R.drawable.bg_main_tab_unselected)

        tabHome.setTextColor(unselectedTextColor)
        tabBluetooth.setTextColor(unselectedTextColor)

        tabHome.setTypeface(null, Typeface.NORMAL)
        tabBluetooth.setTypeface(null, Typeface.NORMAL)

        // Highlight selected
        if (selectedPage == 0) {
            tabHome.setBackgroundResource(R.drawable.bg_main_tab_selected)
            tabHome.setTextColor(selectedTextColor)
            tabHome.setTypeface(null, Typeface.BOLD)
        } else {
            tabBluetooth.setBackgroundResource(R.drawable.bg_main_tab_selected)
            tabBluetooth.setTextColor(selectedTextColor)
            tabBluetooth.setTypeface(null, Typeface.BOLD)
        }
    }
}