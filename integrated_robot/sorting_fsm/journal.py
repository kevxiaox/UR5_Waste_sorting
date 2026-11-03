"""Append-only JSONL journal. The caller chooses the path; task IDs never build filenames."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path

TERMINAL_OUTCOMES = ('HANDED_OFF', 'COMPLETED', 'FAILED', 'TIMED_OUT', 'CANCELLED', 'ABANDONED')
FIELDS = ('timestamp_utc', 'clock_s', 'task_id', 'attempt_id', 'from_state', 'to_state',
          'event', 'command_id', 'reason', 'outcome', 'simulation_only', 'details')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def _reject_constant(name):
    raise ValueError(f'Non-finite number {name} in journal')


class Journal:
    def __init__(self, path, *, clock, wall_clock=utc_now):
        self.path = Path(path)
        self.clock, self.wall_clock = clock, wall_clock

    def write(self, *, event, task_id=None, attempt_id=None, from_state=None, to_state=None,
              command_id=None, reason='', outcome=None, simulation_only=True, details=None):
        clock_s = self.clock()
        if type(clock_s) not in (int, float) or not math.isfinite(clock_s):
            raise ValueError('Clock returned a non-finite value')
        record = dict(zip(FIELDS, (self.wall_clock(), clock_s, task_id, attempt_id, from_state,
                                   to_state, event, command_id, reason, outcome, simulation_only,
                                   details or {})))
        line = json.dumps(record, allow_nan=False, ensure_ascii=False, sort_keys=True)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(line + '\n')
        return record

    def read(self):
        """Strictly parse every record; a corrupt journal must not silently reset deduplication."""
        if not self.path.exists():
            return []
        records = []
        for number, line in enumerate(self.path.read_text(encoding='utf-8').splitlines(), 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line, parse_constant=_reject_constant)
            except (json.JSONDecodeError, ValueError) as error:
                raise ValueError(f'Corrupt journal line {number}: {error}') from None
            if not isinstance(record, dict) or set(record) != set(FIELDS):
                raise ValueError(f'Unexpected journal record at line {number}')
            records.append(record)
        return records

    def task_history(self):
        """{task_id: {'attempts': n, 'last_outcome': str | None}} from accepted submissions."""
        history = {}
        for record in self.read():
            task_id = record['task_id']
            if record['event'] == 'task_accepted':
                entry = history.setdefault(task_id, {'attempts': 0, 'last_outcome': None})
                entry['attempts'] += 1
                entry['last_outcome'] = None
            elif record['outcome'] in TERMINAL_OUTCOMES and task_id in history:
                history[task_id]['last_outcome'] = record['outcome']
        return history
