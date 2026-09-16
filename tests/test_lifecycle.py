"""Real C process/TCP lifecycle tests, with PCI replaced by recording stubs."""
import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class BackendLifecycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            cls.port = probe.getsockname()[1]
        stub = Path(cls.tmp.name) / "pci_stub.c"
        stub.write_text(r'''#include <stdio.h>
#include <stdlib.h>
#include "bxi_pci_drv.h"
int bxi_pci_init(canfd_rx_call cb, void *arg, int cpu) {
    (void)cb; (void)arg; (void)cpu;
    puts("PCI_INIT"); fflush(stdout);
    return getenv("FAIL_PCI_INIT") ? -1 : 0;
}
int bxi_pci_exit(void) { puts("PCI_EXIT"); fflush(stdout); return 0; }
int motor_pwr_set(unsigned int p) { printf("POWER_%u\n", p); fflush(stdout); return 0; }
int canfd_send_packet(canfd_packet *p, unsigned int n) { (void)p; (void)n; return 0; }
''')
        cls.binary = str(Path(cls.tmp.name) / "backend")
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Wpedantic",
                        f"-DSERVER_PORT={cls.port}", "-I", str(ROOT / "backend/bxi_pci_drv"),
                        str(ROOT / "backend/gripper_backend.c"), str(stub),
                        "-pthread", "-lm", "-o", cls.binary], check=True)

    def start(self, managed=True, prefix=(), **kwargs):
        proc = subprocess.Popen(list(prefix) + [self.binary] + (["--managed"] if managed else []),
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, **kwargs)
        self.addCleanup(self.cleanup_process, proc)
        return proc

    @staticmethod
    def cleanup_process(proc):
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for stream in (proc.stdin, proc.stdout):
            if stream and not stream.closed:
                stream.close()

    def connect(self, proc):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                self.fail(proc.stdout.read().decode())
            try:
                sock = socket.create_connection(("127.0.0.1", self.port), timeout=0.2)
                self.addCleanup(sock.close)
                self.receive_until(sock, b"HELLO BXI_GRIPPER_DEMO 1")
                return sock
            except ConnectionRefusedError:
                time.sleep(0.02)
        self.fail("backend did not listen")

    def receive_until(self, sock, marker):
        data = b""
        sock.settimeout(2)
        while marker not in data:
            chunk = sock.recv(4096)
            self.assertTrue(chunk, data)
            data += chunk
        return data

    def assert_clean(self, proc, powered=False):
        self.assertEqual(proc.wait(timeout=3), 0)
        output = proc.stdout.read().decode()
        self.assertEqual(output.count("PCI_INIT"), 1, output)
        self.assertEqual(output.count("PCI_EXIT"), 1, output)
        if powered:
            self.assertIn("POWER_0", output)
            self.assertLess(output.index("POWER_0"), output.index("PCI_EXIT"))
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", self.port))
        return output

    def power_on(self, sock):
        sock.sendall(b"MOTOR_POWER_ON\n")
        self.receive_until(sock, b"Motor power on")

    def test_owner_eof_before_connection(self):
        proc = self.start()
        proc.stdin.close()
        self.assert_clean(proc)

    def test_owner_eof_with_power_on(self):
        proc = self.start()
        sock = self.connect(proc)
        self.power_on(sock)
        proc.stdin.close()
        self.assert_clean(proc, powered=True)

    def test_disconnect_without_shutdown_command(self):
        proc = self.start()
        sock = self.connect(proc)
        self.power_on(sock)
        sock.close()
        self.assert_clean(proc, powered=True)

    def test_shutdown_stops_remaining_commands(self):
        proc = self.start()
        sock = self.connect(proc)
        sock.sendall(b"SHUTDOWN\nMOTOR_POWER_ON\n")
        self.assertNotIn("POWER_1", self.assert_clean(proc))

    def test_env_launcher_preserves_lifetime_pipe(self):
        proc = self.start(prefix=("env", "PATH=/usr/sbin:/usr/bin:/sbin:/bin"))
        sock = self.connect(proc)
        self.power_on(sock)
        proc.stdin.close()
        self.assert_clean(proc, powered=True)

    def test_standalone_disconnect_releases_pci(self):
        proc = self.start(managed=False)
        sock = self.connect(proc)
        sock.close()
        self.assert_clean(proc)

    def test_sigterm_without_client(self):
        proc = self.start()
        # Read readiness from stdout; do not accidentally create a session.
        while b"listening" not in proc.stdout.readline():
            self.assertIsNone(proc.poll())
        proc.terminate()
        self.assertEqual(proc.wait(timeout=3), 0)
        self.assertIn(b"PCI_EXIT", proc.stdout.read())

    def test_port_busy_does_not_initialize_pci(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("127.0.0.1", self.port))
            server.listen()
            proc = self.start()
            self.assertNotEqual(proc.wait(timeout=3), 0)
            self.assertNotIn(b"PCI_INIT", proc.stdout.read())

    def test_driver_init_failure_exits_monitor(self):
        proc = self.start(env=dict(os.environ, FAIL_PCI_INIT="1"))
        self.assertEqual(proc.wait(timeout=3), 1)
        output = proc.stdout.read()
        self.assertIn(b"PCI_INIT", output)
        self.assertNotIn(b"PCI_EXIT", output)


if __name__ == "__main__":
    unittest.main()
