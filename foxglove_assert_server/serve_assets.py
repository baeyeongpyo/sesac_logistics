"""Serve Foxglove 3D Panel assets with browser-compatible CORS headers."""

import argparse
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os


class FoxgloveAssetRequestHandler(SimpleHTTPRequestHandler):
    """Expose static assets to both Foxglove desktop and the web application."""

    _byte_range: tuple[int, int] | None = None

    def end_headers(self) -> None:
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header(
            'Access-Control-Expose-Headers',
            'Accept-Ranges, Content-Length, Content-Range',
        )
        self.send_header('Accept-Ranges', 'bytes')
        super().end_headers()

    @staticmethod
    def _parse_byte_range(value: str, size: int) -> tuple[int, int] | None:
        if not value.startswith('bytes=') or ',' in value:
            return None
        start_text, separator, end_text = value[6:].partition('-')
        if not separator:
            return None
        try:
            if start_text:
                start = int(start_text)
                end = int(end_text) if end_text else size - 1
            elif end_text:
                length = int(end_text)
                if length <= 0:
                    return None
                start = max(size - length, 0)
                end = size - 1
            else:
                return None
        except ValueError:
            return None
        if start < 0 or start >= size or end < start:
            return None
        return start, min(end, size - 1)

    def send_head(self):
        path = self.translate_path(self.path)
        if os.path.isdir(path):
            return super().send_head()
        if path.endswith('/'):
            self.send_error(HTTPStatus.NOT_FOUND, 'File not found')
            return None
        try:
            file_handle = open(path, 'rb')
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND, 'File not found')
            return None

        size = os.fstat(file_handle.fileno()).st_size
        requested_range = self.headers.get('Range')
        selected_range = None
        if requested_range:
            selected_range = self._parse_byte_range(requested_range, size)
            if selected_range is None:
                file_handle.close()
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header('Content-Range', f'bytes */{size}')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return None

        self.send_response(
            HTTPStatus.PARTIAL_CONTENT if selected_range else HTTPStatus.OK,
        )
        self.send_header('Content-type', self.guess_type(path))
        self.send_header('Last-Modified', self.date_time_string(os.path.getmtime(path)))
        if selected_range:
            start, end = selected_range
            self._byte_range = selected_range
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
            self.send_header('Content-Length', str(end - start + 1))
        else:
            self._byte_range = None
            self.send_header('Content-Length', str(size))
        self.end_headers()
        return file_handle

    def copyfile(self, source, outputfile) -> None:
        if self._byte_range is None:
            super().copyfile(source, outputfile)
            return
        start, end = self._byte_range
        source.seek(start)
        remaining = end - start + 1
        while remaining:
            chunk = source.read(min(64 * 1024, remaining))
            if not chunk:
                break
            outputfile.write(chunk)
            remaining -= len(chunk)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description='Serve Foxglove 3D Panel assets over HTTP.',
    )
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--directory', default='/assets')
    arguments = parser.parse_args(argv)

    handler = partial(FoxgloveAssetRequestHandler, directory=arguments.directory)
    with ThreadingHTTPServer((arguments.bind, arguments.port), handler) as server:
        server.serve_forever()


if __name__ == '__main__':
    main()
