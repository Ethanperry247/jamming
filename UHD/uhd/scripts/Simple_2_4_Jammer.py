#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Simple 2.4 GHz Jammer
# Author: jonathan
# GNU Radio version: 3.10.2.0

from gnuradio import analog
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time




class Simple_2_4_Jammer(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Simple 2.4 GHz Jammer", catch_exceptions=True)

        ##################################################
        # Variables
        ##################################################
        self.target_freq = target_freq = 2437e6
        self.sig_sink_input_toggle_switch = sig_sink_input_toggle_switch = 0
        self.sig_sink_bandwidth = sig_sink_bandwidth = 20000000
        self.samp_rate = samp_rate = 3840000
        self.gain_select = gain_select = 70
        self.band2400MHz_bandwidth = band2400MHz_bandwidth = 70000000

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("", '', "master_clock_rate=30.72e6")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            "",
        )
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_sink_0.set_center_freq(target_freq, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_sink_0.set_bandwidth(sig_sink_bandwidth, 0)
        self.uhd_usrp_sink_0.set_gain(gain_select, 0)
        self.blocks_selector_0 = blocks.selector(gr.sizeof_gr_complex*1,sig_sink_input_toggle_switch,0)
        self.blocks_selector_0.set_enabled(True)
        self.analog_wfm_tx_0 = analog.wfm_tx(
        	audio_rate=48000,
        	quad_rate=192000,
        	tau=75e-6,
        	max_dev=1000,
        	fh=-1.0,
        )
        self.analog_sig_source_x_0_0 = analog.sig_source_c(samp_rate, analog.GR_SIN_WAVE, 0, 0, 0, 0)
        self.analog_noise_source_x_0 = analog.noise_source_f(analog.GR_GAUSSIAN, 1, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_noise_source_x_0, 0), (self.analog_wfm_tx_0, 0))
        self.connect((self.analog_sig_source_x_0_0, 0), (self.blocks_selector_0, 0))
        self.connect((self.analog_wfm_tx_0, 0), (self.blocks_selector_0, 1))
        self.connect((self.blocks_selector_0, 0), (self.uhd_usrp_sink_0, 0))


    def get_target_freq(self):
        return self.target_freq

    def set_target_freq(self, target_freq):
        self.target_freq = target_freq
        self.uhd_usrp_sink_0.set_center_freq(self.target_freq, 0)

    def get_sig_sink_input_toggle_switch(self):
        return self.sig_sink_input_toggle_switch

    def set_sig_sink_input_toggle_switch(self, sig_sink_input_toggle_switch):
        self.sig_sink_input_toggle_switch = sig_sink_input_toggle_switch
        self.blocks_selector_0.set_input_index(self.sig_sink_input_toggle_switch)

    def get_sig_sink_bandwidth(self):
        return self.sig_sink_bandwidth

    def set_sig_sink_bandwidth(self, sig_sink_bandwidth):
        self.sig_sink_bandwidth = sig_sink_bandwidth
        self.uhd_usrp_sink_0.set_bandwidth(self.sig_sink_bandwidth, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_sig_source_x_0_0.set_sampling_freq(self.samp_rate)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_gain_select(self):
        return self.gain_select

    def set_gain_select(self, gain_select):
        self.gain_select = gain_select
        self.uhd_usrp_sink_0.set_gain(self.gain_select, 0)

    def get_band2400MHz_bandwidth(self):
        return self.band2400MHz_bandwidth

    def set_band2400MHz_bandwidth(self, band2400MHz_bandwidth):
        self.band2400MHz_bandwidth = band2400MHz_bandwidth




def main(top_block_cls=Simple_2_4_Jammer, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
