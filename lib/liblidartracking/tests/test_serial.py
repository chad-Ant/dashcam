"""Real POSIX tty path against a pseudo-terminal, never a physical device."""
import os
import pty
import time
import unittest
from lidartracking.bridge import SerialBridge, LatencyBounds, crc_line


class SerialTests(unittest.TestCase):
    def test_requires_bench_opt_in(self):
        with self.assertRaises(ValueError): SerialBridge("/must/not/be/opened")

    def test_roundtrip_and_fragmented_measurement(self):
        master,slave=pty.openpty()
        bridge=None
        try:
            bridge=SerialBridge(os.ttyname(slave),bench_firmware=True,latency=LatencyBounds(.001,.002))
            os.set_blocking(master,False)
            bridge.poll()
            query=os.read(master,100)
            self.assertEqual(query,b"Q1 00000001\n")
            os.write(master,crc_line(b"T1 00000001 000F4240"))
            time.sleep(.002)
            self.assertEqual(bridge.poll(),[])
            self.assertIsNotNone(bridge.decoder.clock.anchor)
            line=crc_line(b"L1 00000001 000F4240 00002710 00000019 00002710 00000019")
            os.write(master,line[:20]); time.sleep(.002)
            self.assertEqual(bridge.poll(),[])
            os.write(master,line[20:]); time.sleep(.002)
            results=bridge.poll()
            self.assertEqual(len(results),1)
            self.assertTrue(results[0].timing_verified)
            self.assertEqual(results[0].first_m,100.)
            os.write(master,b"x"*200+b"\n"+line); time.sleep(.002)
            self.assertEqual(bridge.poll(),[]) # duplicate sequence, not a new sample
            self.assertLessEqual(len(bridge.buffer),62)
            os.write(master,crc_line(b"S1 000F4240 00000001 00050200 00000000 00000002 00000003"))
            time.sleep(.002)
            bridge.poll()
            self.assertEqual(bridge.status["firmware"],0x50200)
            self.assertEqual(bridge.status["parse_errors"],3)
        finally:
            if bridge: bridge.close()
            os.close(slave); os.close(master)
