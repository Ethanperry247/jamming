#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Simple 2.4 GHz Receiver
# Author: jonathan
# GNU Radio version: 3.10.2.0

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




class Simple_2_4_Receiver(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Simple 2.4 GHz Receiver", catch_exceptions=True)

        ##################################################
        # Variables
        ##################################################
        self.target_freq = target_freq = 2437e6
        self.samp_rate = samp_rate = 3840000

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
            ",".join(("", '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_source_0.set_center_freq(target_freq, 0)
        self.uhd_usrp_source_0.set_antenna("RX2", 0)
        self.uhd_usrp_source_0.set_bandwidth(20000000, 0)
        self.uhd_usrp_source_0.set_gain(50, 0)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_null_sink_0, 0))


    def get_target_freq(self):
        return self.target_freq

    def set_target_freq(self, target_freq):
        self.target_freq = target_freq
        self.uhd_usrp_source_0.set_center_freq(self.target_freq, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)

    def get_usrp_sensor_name(self):
        return self.uhd_usrp_source_0.get_sensor_names(0)

    def get_usrp_info(self):
        return self.uhd_usrp_source_0.get_usrp_info(0)

    def get_usrp_sensor(self):
        return self.uhd_usrp_source_0.get_sensor(self.get_usrp_sensor_name()[1],0)

    def get_usrp_finite_acquisition(self, n):
        return self.uhd_usrp_source_0.finite_acquisition_v(n)

    def activate_usrp_jammer(self):
        self.set_sig_sink_input_toggle_switch(1)

    def deactivate_usrp_jammer(self):
        self.set_sig_sink_input_toggle_switch(0)




def main(top_block_cls=Simple_2_4_Receiver, options=None):
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
