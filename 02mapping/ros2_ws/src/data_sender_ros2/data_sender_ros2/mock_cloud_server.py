import json
from http.server import BaseHTTPRequestHandler, HTTPServer


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(content_length)
        payload = json.loads(body.decode("utf-8"))
        if self.path == "/receive_ros2":
            vehicle_id = payload.get("vehicle_id", "unknown")
            topic = payload.get("cloud_topic", "")
            shape = payload.get("shape", {})
            print(
                f"receive_ros2 vehicle={vehicle_id} topic={topic} "
                f"height={shape.get('height')} width={shape.get('width')}"
                ,
                flush=True,
            )
        elif self.path == "/sync_gps":
            print(
                f"sync_gps vehicle={payload.get('vehicle_id')} "
                f"x={payload.get('x')} y={payload.get('y')} theta={payload.get('theta')}",
                flush=True,
            )
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"ok")

    def log_message(self, format, *args):
        return


def main():
    server = HTTPServer(("0.0.0.0", 5000), Handler)
    print("mock_cloud_server listening on 0.0.0.0:5000", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
