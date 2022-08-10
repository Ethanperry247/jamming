#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Simple 2.4 GHz Transceiver
# Author: jonathan
# GNU Radio version: 3.8.3.1

from gnuradio import analog
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time


class Simple_2_4_Transceiver(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Simple 2.4 GHz Transceiver")

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
        self.uhd_usrp_source_1 = uhd.usrp_source(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_1.set_center_freq(target_freq, 0)
        self.uhd_usrp_source_1.set_gain(50, 0)
        self.uhd_usrp_source_1.set_antenna('RX2', 0)
        self.uhd_usrp_source_1.set_bandwidth(20000000, 0)
        self.uhd_usrp_source_1.set_samp_rate(samp_rate)
        self.uhd_usrp_source_1.set_time_unknown_pps(uhd.time_spec())
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("", '', "master_clock_rate=30.72e6")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            "",
        )
        self.uhd_usrp_sink_0.set_center_freq(target_freq, 0)
        self.uhd_usrp_sink_0.set_gain(gain_select, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_sink_0.set_bandwidth(sig_sink_bandwidth, 0)
        self.uhd_usrp_sink_0.set_clock_rate(30.72e6, uhd.ALL_MBOARDS)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec())
        self.blocks_selector_0 = blocks.selector(gr.sizeof_gr_complex*1,sig_sink_input_toggle_switch,0)
        self.blocks_selector_0.set_enabled(True)
        self.blocks_null_sink_0_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
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
        self.connect((self.uhd_usrp_source_1, 0), (self.blocks_null_sink_0_0, 0))


    def get_target_freq(self):
        return self.target_freq

    def set_target_freq(self, target_freq):
        self.target_freq = target_freq
        self.uhd_usrp_sink_0.set_center_freq(self.target_freq, 0)
        self.uhd_usrp_source_1.set_center_freq(self.target_freq, 0)

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
        self.uhd_usrp_source_1.set_samp_rate(self.samp_rate)

    def get_gain_select(self):
        return self.gain_select

    def set_gain_select(self, gain_select):
        self.gain_select = gain_select
        self.uhd_usrp_sink_0.set_gain(self.gain_select, 0)

    def get_band2400MHz_bandwidth(self):
        return self.band2400MHz_bandwidth

    def set_band2400MHz_bandwidth(self, band2400MHz_bandwidth):
        self.band2400MHz_bandwidth = band2400MHz_bandwidth

    def get_usrp_sensor_name(self):
        return self.uhd_usrp_source_1.get_sensor_names(0)

    def get_usrp_info(self):
        return self.uhd_usrp_source_1.get_usrp_info(0)

    def get_usrp_sensor(self):
        return self.uhd_usrp_source_1.get_sensor(self.get_usrp_sensor_name()[1],0)

    def get_usrp_finite_acquisition(self, n):
        return self.uhd_usrp_source_1.finite_acquisition_v(n)

    def activate_usrp_jammer(self):
        self.set_sig_sink_input_toggle_switch(1)

    def deactivate_usrp_jammer(self):
        self.set_sig_sink_input_toggle_switch(0)





def main(top_block_cls=Simple_2_4_Transceiver, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        tb.deactivate_usrp_jammer()
        input('Block info:')
        print(tb.get_usrp_info())
        print('Sensor names: ', tb.get_usrp_sensor_name())
        input('Sensor: ')
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print('Make acquisition: ')
        print(tb.get_usrp_finite_acquisition(5))
        tb.activate_usrp_jammer()
        input('Sensor: ')
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print(tb.get_usrp_sensor())
        print('Make acquisition: ')
        print(tb.get_usrp_finite_acquisition(5))
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
