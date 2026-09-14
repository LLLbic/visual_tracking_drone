import unittest
from uav_preview.control_transport import GuardedControlSocket
from uav_preview.safety import SafetyGate


class FakeSocket:
    def __init__(self,*_): self.sent=[]
    def settimeout(self,seconds): self.timeout=seconds
    def sendto(self,p,e): self.sent.append(p); return len(p)
    def close(self): pass


class TransportTests(unittest.TestCase):
    def test_latch_blocks_all_continuous_producers(self):
        gate=SafetyGate(initially_latched=True)
        sockets=[GuardedControlSocket(gate.run_if_unlatched,2,2,FakeSocket) for _ in range(3)]
        for s in sockets:
            with self.assertRaises(OSError): s.sendto(b'old',('127.0.0.1',1))
        gate.reset_estop()
        for s in sockets: s.sendto(b'allowed',('127.0.0.1',1))
        gate.latch_estop()
        for s in sockets:
            with self.assertRaises(OSError): s.sendto(b'queued',('127.0.0.1',1))
            self.assertEqual(s._socket.sent,[b'allowed'])
