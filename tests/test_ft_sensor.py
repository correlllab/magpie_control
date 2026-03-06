"""Tests for ft_sensor.py.

Unit tests run without hardware.
Integration tests (marked 'integration') require the sensor at 192.168.0.3.

Run all:        pytest magpie_control/tests/test_ft_sensor.py -v
Run unit only:  pytest magpie_control/tests/test_ft_sensor.py -v -m "not integration"
Run hw only:    pytest magpie_control/tests/test_ft_sensor.py -v -m integration
"""

import struct
import time
import pytest
from magpie_control.ft_sensor import OptoForce, OptoForceCmd, RESPONS_SZ, FORCE_DIV, TORQUE_DIV

SENSOR_IP = "192.168.0.3"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_packet(fx=0, fy=0, fz=0, tx=0, ty=0, tz=0) -> bytes:
    """Build a valid 36-byte response packet with given raw integer values."""
    # DG_res = '! 3I 6i'  → 3 uint32 header fields + 6 int32 data fields
    return struct.pack('! 3I 6i', 0, 0, 0, fx, fy, fz, tx, ty, tz)


# ---------------------------------------------------------------------------
# Unit tests (no hardware)
# ---------------------------------------------------------------------------

class TestUnpacking:
    def test_response_size(self):
        pkt = make_packet()
        assert len(pkt) == RESPONS_SZ

    def test_force_raw_values(self):
        pkt = make_packet(fx=10000, fy=-5000, fz=3000)
        vals = OptoForceCmd.unpack_response(pkt)
        assert vals == [10000, -5000, 3000, 0, 0, 0]

    def test_torque_raw_values(self):
        pkt = make_packet(tx=100000, ty=-50000, tz=25000)
        vals = OptoForceCmd.unpack_response(pkt)
        assert vals == [0, 0, 0, 100000, -50000, 25000]

    def test_recv_datum_scaling(self, monkeypatch):
        """recv_datum should scale forces by FORCE_DIV and torques by TORQUE_DIV."""
        pkt = make_packet(fx=int(FORCE_DIV), fy=int(-FORCE_DIV),
                          tx=int(TORQUE_DIV), tz=int(-TORQUE_DIV * 2))
        sensor = OptoForce.__new__(OptoForce)
        sensor.cmd = OptoForceCmd()

        class FakeSock:
            def recvfrom(self, n):
                return pkt, None
        sensor.sock_r = FakeSock()

        result = sensor.recv_datum()
        assert result[0] == pytest.approx(1.0)   # Fx
        assert result[1] == pytest.approx(-1.0)  # Fy
        assert result[2] == pytest.approx(0.0)   # Fz
        assert result[3] == pytest.approx(1.0)   # Tx
        assert result[4] == pytest.approx(0.0)   # Ty
        assert result[5] == pytest.approx(-2.0)  # Tz

    def test_recv_datum_timeout_returns_empty(self, monkeypatch):
        import socket
        sensor = OptoForce.__new__(OptoForce)
        sensor.cmd = OptoForceCmd()

        class FakeSock:
            def recvfrom(self, n):
                raise socket.timeout
        sensor.sock_r = FakeSock()

        assert sensor.recv_datum() == []

    def test_recv_datum_wrong_size_returns_empty(self):
        sensor = OptoForce.__new__(OptoForce)
        sensor.cmd = OptoForceCmd()

        class FakeSock:
            def recvfrom(self, n):
                return b'\x00' * 10, None  # wrong size
        sensor.sock_r = FakeSock()

        assert sensor.recv_datum() == []


class TestSpeedCommand:
    @pytest.mark.parametrize("hz,expected_reg", [
        (500, 2),
        (250, 4),
        (100, 10),
        (50,  20),
        (20,  50),
        (10, 100),
        (5,  200),
    ])
    def test_exact_valid_rates(self, hz, expected_reg):
        cmd = OptoForceCmd.make_set_speed(hz)
        _, opcode, speed_reg = struct.unpack('! 2H I', cmd)
        assert opcode == 0x0082
        assert speed_reg == expected_reg

    def test_snaps_to_nearest(self):
        # 80 Hz is between 50 and 100; nearest is 100
        cmd = OptoForceCmd.make_set_speed(80)
        _, _, speed_reg = struct.unpack('! 2H I', cmd)
        assert speed_reg == 10  # 100 Hz

    def test_clamps_above_max(self):
        cmd = OptoForceCmd.make_set_speed(1000)
        _, _, speed_reg = struct.unpack('! 2H I', cmd)
        assert speed_reg == 2  # snaps to 500 Hz max


# ---------------------------------------------------------------------------
# Integration tests (live sensor required)
# ---------------------------------------------------------------------------

@pytest.mark.integration
class TestSpeedProbe:
    """Empirically probe which speed_reg values the sensor actually accepts.

    For each candidate, send the speed command, flush stale packets, read for
    a fixed window, and report the measured Hz.  This tells us:
      - Which speed_reg values produce a measurable rate change
      - Whether the sensor silently ignores invalid values (stays at last good rate)
      - What the practical ceiling is over this UDP link

    Run with: pytest -v -m integration -s  (the -s keeps the print output)
    """

    # speed_reg values to probe: 1000/speed_reg = requested Hz
    # Covers 1 Hz … 1000 Hz in decade-ish steps plus the two originally hardcoded values
    SPEED_REGS = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]

    def _measure_rate_for_speed_reg(self, sensor: OptoForce, speed_reg: int,
                                    window_s: float = 1.0) -> float:
        """Send speed command, flush, count packets received in window_s seconds."""
        cmd = OptoForceCmd.DG_cmd.pack(0x1234, 0x0082, speed_reg)
        sensor.send_datagram(cmd)  # includes 20ms settle wait
        sensor.flush()

        received = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < window_s:
            if sensor.recv_datum():
                received += 1
        elapsed = time.perf_counter() - t0
        return received / elapsed

    def test_probe_all_speeds(self):
        """Sweep speed_reg values and print actual measured rates.

        Does not assert a specific rate — the output is the result.
        Fails only if the sensor returns no data at all for any setting.
        """
        sensor = OptoForce(ip_address=SENSOR_IP, poll_rate=100)
        sensor.connect()

        results = {}
        print("\n\nspeed_reg | requested Hz | measured Hz")
        print("----------|--------------|------------")
        for sr in self.SPEED_REGS:
            requested = 1000 / sr
            actual = self._measure_rate_for_speed_reg(sensor, sr)
            results[sr] = actual
            print(f"  {sr:>6}  |   {requested:>8.1f}   |  {actual:>7.1f}")

        sensor.close()

        # Every speed_reg should yield at least some data
        for sr, hz in results.items():
            assert hz > 0, f"speed_reg={sr} produced no data — sensor may have stalled"

    def test_detects_rate_change_between_50_and_100hz(self):
        """Verify the sensor actually changes rate between the two originally supported speeds."""
        sensor = OptoForce(ip_address=SENSOR_IP, poll_rate=100)
        sensor.connect()

        hz_100 = self._measure_rate_for_speed_reg(sensor, speed_reg=10)  # 1000/10 = 100 Hz
        hz_50  = self._measure_rate_for_speed_reg(sensor, speed_reg=20)  # 1000/20 = 50  Hz
        sensor.close()

        print(f"\n100Hz cmd → {hz_100:.1f} Hz,  50Hz cmd → {hz_50:.1f} Hz")
        # If the sensor responds to speed commands, the 100Hz setting should be
        # measurably faster than the 50Hz setting (allow ±20% slop)
        assert hz_100 > hz_50 * 1.2, (
            f"Expected 100Hz setting to be >20% faster than 50Hz setting, "
            f"but got {hz_100:.1f} vs {hz_50:.1f} Hz — sensor may ignore speed commands"
        )


@pytest.mark.integration
class TestLiveRate:
    """Measure actual read rate from hardware. Requires sensor at SENSOR_IP."""

    def _measure_hz(self, poll_rate: int, n_samples: int = 300) -> float:
        sensor = OptoForce(ip_address=SENSOR_IP, poll_rate=poll_rate)
        sensor.connect()
        sensor.flush()

        received = 0
        t0 = time.perf_counter()
        for _ in range(n_samples):
            if sensor.recv_datum():
                received += 1
        elapsed = time.perf_counter() - t0
        sensor.close()

        hz = received / elapsed
        print(f"\npoll_rate={poll_rate} Hz → got {received}/{n_samples} in "
              f"{elapsed:.3f}s = {hz:.1f} Hz actual")
        return hz

    def test_rate_100hz(self):
        hz = self._measure_hz(poll_rate=100)
        assert hz >= 80, f"Expected ≥80 Hz at poll_rate=100, got {hz:.1f} Hz"

    def test_rate_1000hz(self):
        """Push sensor to max; realistically expect 200+ Hz over UDP."""
        hz = self._measure_hz(poll_rate=1000, n_samples=500)
        assert hz >= 100, f"Expected ≥100 Hz at poll_rate=1000, got {hz:.1f} Hz"

    def test_no_stale_data(self):
        """After flush(), timestamps should advance monotonically — no repeated packets."""
        sensor = OptoForce(ip_address=SENSOR_IP, poll_rate=100)
        sensor.connect()
        sensor.flush()

        readings = []
        t_prev = time.perf_counter()
        for _ in range(50):
            d = sensor.recv_datum()
            if d:
                readings.append((time.perf_counter(), d))
        sensor.close()

        gaps = [readings[i+1][0] - readings[i][0] for i in range(len(readings)-1)]
        max_gap = max(gaps)
        mean_gap = sum(gaps) / len(gaps)
        print(f"\nGap mean={mean_gap*1000:.1f}ms  max={max_gap*1000:.1f}ms")
        # No gap should exceed 3× the expected period (signs of stale drain)
        assert max_gap < 3 / 100, f"Max gap {max_gap*1000:.1f}ms suggests stale buffer"
