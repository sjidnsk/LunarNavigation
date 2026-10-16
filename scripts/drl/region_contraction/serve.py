"""Serve the experiment viewer and read-only artifacts on localhost."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class ViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, root, **kwargs):
        self.artifact_root = Path(root).resolve()
        super().__init__(*args, directory=str(self.artifact_root), **kwargs)

    def translate_path(self, path):
        if path.split('?', 1)[0] in ('/', '/index.html'):
            return str(Path(__file__).with_name('viewer.html'))
        if path.startswith('/data/'):
            path = path[5:]
        return super().translate_path(path)

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--port', type=int, default=39136)
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), partial(ViewerHandler, root=args.root))
    print(f'http://127.0.0.1:{args.port}', flush=True)
    server.serve_forever()
