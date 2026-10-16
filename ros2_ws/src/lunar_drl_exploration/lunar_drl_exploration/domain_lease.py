"""Host-local DDS domain ownership, including older unleased ROS children."""
import fcntl
import os
from pathlib import Path


class DomainBusy(RuntimeError):
    pass


class DomainLease:
    def __init__(self, domain_id, fd):
        self.domain_id, self.fd = domain_id, fd

    @classmethod
    def acquire(cls, domain_id, *, root=None, proc_root=Path('/proc')):
        root = Path(root) if root is not None else Path.home()/'.cache/lunar-drl-training/dds-domains'
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        candidates = range(232, 219, -1) if domain_id is None else (domain_id,)
        occupied = {}
        proc_root = Path(proc_root)
        if proc_root.exists():
            for process in proc_root.iterdir():
                if not process.name.isdecimal() or int(process.name) == os.getpid():
                    continue
                try:
                    values = (process/'environ').read_bytes().split(b'\0')
                except (OSError, PermissionError):
                    continue
                for value in values:
                    if value.startswith(b'ROS_DOMAIN_ID='):
                        try:
                            occupied.setdefault(int(value.split(b'=', 1)[1]), []).append(process.name)
                        except ValueError:
                            pass
        for candidate in candidates:
            if type(candidate) is not int or not 0 <= candidate <= 232:
                raise ValueError('invalid isolated DDS domain')
            if candidate in occupied:
                if domain_id is not None:
                    raise DomainBusy(f'DDS domain {candidate} occupied by processes {occupied[candidate]}')
                continue
            fd = os.open(root/f'domain-{candidate}.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(fd)
                if domain_id is not None:
                    raise DomainBusy(f'DDS domain {candidate} already leased') from None
                continue
            os.ftruncate(fd, 0)
            os.write(fd, f'{os.getpid()}\n'.encode())
            return cls(candidate, fd)
        raise DomainBusy('no free isolated evaluation DDS domain in 220..232')

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
