"""USB auto-selection tests; fake sysfs only, no device is opened."""
import pathlib
import subprocess
import tempfile
import unittest

SELECTOR = pathlib.Path(__file__).resolve().parents[2] / "find_port.sh"


class PortTests(unittest.TestCase):
    def select(self, devices):
        with tempfile.TemporaryDirectory(prefix="mkr-port-test-") as tmp:
            root = pathlib.Path(tmp)
            for number, (vid, pid) in enumerate(devices):
                usb = root / f"usb{number}"
                interface = usb / "interface"
                interface.mkdir(parents=True)
                (usb / "idVendor").write_text(vid + "\n")
                (usb / "idProduct").write_text(pid + "\n")
                tty = root / f"ttyACM{number}"
                tty.mkdir()
                (tty / "device").symlink_to(interface)
            return subprocess.run(
                ["bash", "-c", 'source "$1"; find_mkr_zero_port "$2"',
                 "port-test", str(SELECTOR), str(root)],
                text=True, capture_output=True, timeout=10)

    def test_no_board(self):
        self.assertNotEqual(self.select([]).returncode, 0)

    def test_wrong_arduino_and_c3(self):
        self.assertNotEqual(self.select([("2341", "804e"), ("303a", "1001")]).returncode, 0)

    def test_product_id_selects_mkr(self):
        r = self.select([("2341", "804e"), ("2341", "804f")])
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.strip(), "/dev/ttyACM1")

    def test_bootloader(self):
        self.assertEqual(self.select([("2341", "004f")]).returncode, 0)

    def test_ambiguous_boards(self):
        self.assertNotEqual(self.select([("2341", "804f"), ("2341", "004f")]).returncode, 0)


if __name__ == "__main__":
    unittest.main()
