package com.example.mdpgroup24.adapters

import androidx.fragment.app.Fragment
import androidx.viewpager2.adapter.FragmentStateAdapter
import com.example.mdpgroup24.fragments.ChallengeFragment
import com.example.mdpgroup24.fragments.ChatFragment
import com.example.mdpgroup24.fragments.ConfigurationFragment
import com.example.mdpgroup24.fragments.ControlFragment
import com.example.mdpgroup24.fragments.HomeFragment

class SubTabPagerAdapter(
    fragment: HomeFragment
) : FragmentStateAdapter(fragment) {

    override fun getItemCount(): Int {
        return 4
    }

    override fun createFragment(position: Int): Fragment {
        return when (position) {
            0 -> ConfigurationFragment()
            1 -> ControlFragment()
            2 -> ChatFragment()
            3 -> ChallengeFragment()
            else -> ConfigurationFragment()
        }
    }
}