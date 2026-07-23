#!/usr/bin/env python3
"""Security-focused unit tests for the bundled dashcam control client."""

import hashlib
import hmac
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock


CLIENT_PATH = Path(__file__).resolve().parents[1] / "tools" / "dashcam_ctl.py"
SPEC = importlib.util.spec_from_file_location("dashcam_ctl", CLIENT_PATH)
dashcam_ctl = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dashcam_ctl)


class FakeSocket:
    def __init__(self, replies):
        self._replies = list(replies)
        self.sent = []

    def recv(self, _size):
        return self._replies.pop(0) if self._replies else b""

    def sendall(self, data):
        self.sent.append(data)


class DashcamCtlSecurityTest(unittest.TestCase):
    def test_private_token_file_is_accepted(self):
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8") as stream:
            os.chmod(stream.name, 0o600)
            stream.write("test-psk\n")
            stream.flush()
            self.assertEqual(dashcam_ctl._read_token_file(stream.name), "test-psk")

    def test_permissive_token_file_is_rejected(self):
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8") as stream:
            stream.write("test-psk\n")
            stream.flush()
            os.chmod(stream.name, 0o640)
            with self.assertRaisesRegex(ValueError, "group or other"):
                dashcam_ctl._read_token_file(stream.name)

    def test_token_file_symlink_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "token"
            target.write_text("test-psk\n", encoding="utf-8")
            target.chmod(0o600)
            link = Path(directory) / "token-link"
            link.symlink_to(target)
            with self.assertRaises(OSError):
                dashcam_ctl._read_token_file(link)

    def test_challenge_prompts_and_sends_expected_hmac(self):
        nonce = "0123456789abcdef"
        sock = FakeSocket([
            ("AUTH-CHALLENGE " + nonce + "\n").encode("ascii"),
            b"AUTH-OK\n",
        ])
        with mock.patch.object(dashcam_ctl, "_prompt_token",
                               return_value="test-psk") as prompt:
            ok, remaining = dashcam_ctl._handshake(sock)

        expected = hmac.new(
            b"test-psk", nonce.encode("ascii"), hashlib.sha256).hexdigest()
        self.assertTrue(ok)
        self.assertEqual(remaining, b"")
        self.assertEqual(sock.sent, [("AUTH " + expected + "\n").encode("ascii")])
        prompt.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
