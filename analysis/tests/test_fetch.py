import threading
from datetime import date
from http.server import BaseHTTPRequestHandler, HTTPServer

from synth import write_run

from slideranalysis.fetch import fetch, next_seq, sanitize


def _serve(body: bytes):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/lab/dump.csv":
                self.send_response(200)
                self.send_header("Content-Type", "text/csv")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def test_fetch_names_sequences_and_notes(tmp_path):
    src = write_run(tmp_path / "src.csv", n_ticks=50, experiment="noise_floor")
    body = src.read_bytes()
    server = _serve(body)
    try:
        host = f"127.0.0.1:{server.server_address[1]}"
        data_dir = tmp_path / "data"

        dest1 = fetch(host, data_dir=data_dir, note="wifi on, knob at 50%")
        assert dest1.parent.name == date.today().isoformat()
        assert dest1.name == "01_noise_floor.csv"
        assert dest1.read_bytes() == body

        dest2 = fetch(host, data_dir=data_dir)
        assert dest2.name == "02_noise_floor.csv"

        notes = (dest1.parent / "notes.md").read_text()
        assert "wifi on, knob at 50%" in notes
        assert "01_noise_floor.csv" in notes
    finally:
        server.shutdown()


def test_next_seq_and_sanitize(tmp_path):
    assert next_seq(tmp_path) == "01"
    (tmp_path / "01_a.csv").touch()
    (tmp_path / "03_b.csv").touch()
    assert next_seq(tmp_path) == "02"  # first free number, not max+1
    assert sanitize("pwm_coupling(0.2,fwd)") == "pwm_coupling_0.2_fwd"
    assert sanitize("") == "run"
