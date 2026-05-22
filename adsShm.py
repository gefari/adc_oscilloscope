import ctypes
import posix_ipc
import mmap

SHM_SAMPLES = 4096

class AdsShm(ctypes.Structure):
    _fields_ = [
        ("pair0",              ctypes.c_float * SHM_SAMPLES),
        ("pair0_head",         ctypes.c_uint),
        ("pair0_tail",         ctypes.c_uint),
        ("pair0_lost",         ctypes.c_uint),
        ("current_pair0_raw",  ctypes.c_int),

        ("pair1",              ctypes.c_float * SHM_SAMPLES),
        ("pair1_head",         ctypes.c_uint),
        ("pair1_tail",         ctypes.c_uint),
        ("pair1_lost",         ctypes.c_uint),
        ("current_pair1_raw",  ctypes.c_int),

        ("sample_count",       ctypes.c_uint),
        ("overruns",           ctypes.c_uint),
        ("sample_rate",        ctypes.c_uint),
        ("cmd_dr",             ctypes.c_uint),
        ("cmd_filter",         ctypes.c_uint),
        ("cmd_gain",           ctypes.c_uint),
        ("cmd_refmux",         ctypes.c_uint),
        ("cmd_inpmux",         ctypes.c_uint),
        ("cmd_raw_wreg",       ctypes.c_uint),
        ("cmd_conv",           ctypes.c_uint),
        ("rb_mode0",           ctypes.c_uint),
        ("rb_mode1",           ctypes.c_uint),
        ("rb_mode2",           ctypes.c_uint),
        ("rb_refmux",          ctypes.c_uint),
        ("rb_inpmux",          ctypes.c_uint),
    ]


class ShmReader:

    SHM_NAME    = "/ads1263"
    SHM_SAMPLES = SHM_SAMPLES

    def __init__(self):
        self.mem   = posix_ipc.SharedMemory(self.SHM_NAME)
        self._mmap = mmap.mmap(self.mem.fd, ctypes.sizeof(AdsShm))
        self.mem.close_fd()
        # Map struct directly onto mmap — reads AND writes go to shared memory
        self._buf  = (ctypes.c_byte * ctypes.sizeof(AdsShm)).from_buffer(self._mmap)
        self.shm   = AdsShm.from_buffer(self._buf)

    # ── pair0 consumer ───────────────────────────────────────

    def available_pair0(self):
        """Number of unread samples in the pair0 ring buffer."""
        h = self.shm.pair0_head
        t = self.shm.pair0_tail
        return (h - t) % SHM_SAMPLES

    def read_next_pair0(self):
        """
        Read the next unread sample from pair0.
        Returns (value_volts, lost_so_far) or None if the buffer is empty.
        Advances the tail so the producer knows the slot is free.
        """
        h = self.shm.pair0_head
        t = self.shm.pair0_tail
        if h == t:
            return None
        value = self.shm.pair0[t]
        lost  = self.shm.pair0_lost
        self.shm.pair0_tail = (t + 1) % SHM_SAMPLES
        return value, lost

    def drain_pair0(self):
        """Return all available pair0 samples as a list of floats, oldest first."""
        samples = []
        while True:
            r = self.read_next_pair0()
            if r is None:
                break
            samples.append(r[0])
        return samples

    # ── pair1 consumer ───────────────────────────────────────

    def available_pair1(self):
        h = self.shm.pair1_head
        t = self.shm.pair1_tail
        return (h - t) % SHM_SAMPLES

    def read_next_pair1(self):
        h = self.shm.pair1_head
        t = self.shm.pair1_tail
        if h == t:
            return None
        value = self.shm.pair1[t]
        lost  = self.shm.pair1_lost
        self.shm.pair1_tail = (t + 1) % SHM_SAMPLES
        return value, lost

    def drain_pair1(self):
        samples = []
        while True:
            r = self.read_next_pair1()
            if r is None:
                break
            samples.append(r[0])
        return samples

    # ── status ───────────────────────────────────────────────

    def read_sample_count(self):
        return self.shm.sample_count

    def read_overruns(self):
        return self.shm.overruns

    def read_lost(self):
        return self.shm.pair0_lost, self.shm.pair1_lost
