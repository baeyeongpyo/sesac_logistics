"""Offline shell tests: ROS, signals, process discovery and sleeps are mocked."""
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = [ROOT / 'tools/stopper.sh']

HARNESS = r'''
source "$1"
scenario="$2"
declare -A alive=([101]=1 [102]=1 [103]=1)
supervisor_patterns=(supervisor)
worker_patterns=(worker)
motor_patterns=(motor)
sleep() { :; }
live_pids() {
    local pid
    case "$1" in
        supervisor) pid=101;;
        worker) pid=102;;
        motor) pid=103;;
        *) return 2;;
    esac
    [[ ${alive[$pid]} == 0 ]] || echo "$pid"
}
kill() {
    local sig="$1" pid="$2"
    echo "SIGNAL $sig $pid"
    if [[ "$scenario" == worker_failure && "$pid" == 102 ]] ||
       [[ "$scenario" == supervisor_failure && "$pid" == 101 ]]; then
        return 1
    fi
    alive[$pid]=0
}
timeout() {
    if [[ "$*" == *motor_direct_stop.py* ]]; then
        echo "SERIAL" >&2
        [[ ${alive[101]} == 0 && ${alive[102]} == 0 && ${alive[103]} == 0 ]]
        return
    fi
    echo "FINAL $*" >&2
    [[ ${alive[101]} == 0 && ${alive[102]} == 0 && ${alive[103]} == 1 ]] || return 90
    [[ "$scenario" != publish_failure ]]
}
shutdown_stack
result=$?
echo "RESULT $result MOTOR ${alive[103]}"
'''


class StopperOrderTests(unittest.TestCase):
    def run_case(self, scenario):
        for script in SCRIPTS:
            with self.subTest(script=script.name, scenario=scenario):
                result = subprocess.run(
                    ['bash', '-c', HARNESS, 'test', str(script), scenario],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                    check=True,
                )
                yield result.stdout

    def test_motor_is_last_and_final_zero_waits_for_subscriber(self):
        for output in self.run_case('success'):
            self.assertLess(output.index('SIGNAL -KILL 101'), output.index('SIGNAL -INT 102'))
            self.assertLess(output.index('SIGNAL -INT 102'), output.index('FINAL '))
            self.assertLess(output.index('FINAL '), output.index('SIGNAL -INT 103'))
            self.assertIn('--wait-matching-subscriptions 1', output)
            self.assertIn('--times 20', output)
            self.assertIn('RESULT 0 MOTOR 0', output)

    def test_failed_publication_uses_serial_fallback(self):
        for output in self.run_case('publish_failure'):
            self.assertIn('RESULT 0 MOTOR 0', output)
            self.assertIn('SERIAL', output)

    def test_surviving_worker_preserves_motor_driver(self):
        for output in self.run_case('worker_failure'):
            self.assertIn('RESULT 1 MOTOR 1', output)
            self.assertNotIn('FINAL ', output)

    def test_surviving_supervisor_prevents_cascaded_shutdown(self):
        for output in self.run_case('supervisor_failure'):
            self.assertIn('RESULT 1 MOTOR 1', output)
            self.assertNotIn('SIGNAL -INT 102', output)
            self.assertNotIn('FINAL ', output)


if __name__ == '__main__':
    unittest.main()
