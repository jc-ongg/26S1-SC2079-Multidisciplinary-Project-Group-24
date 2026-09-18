package com.example.mdpgroup24.adapters

import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentActivity
import androidx.viewpager2.adapter.FragmentStateAdapter
import com.example.mdpgroup24.fragments.BluetoothFragment
import com.example.mdpgroup24.fragments.HomeFragment

class MainPagerAdapter(
    activity: FragmentActivity
) : FragmentStateAdapter(activity) {

    override fun getItemCount(): Int {
        return 2
    }

    override fun createFragment(position: Int): Fragment {
        return when (position) {
            0 -> HomeFragment()
            1 -> BluetoothFragment()
            else -> HomeFragment()
        }
    }
}