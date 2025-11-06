import http.server
import socketserver
import webbrowser
import os
from pathlib import Path

def start_server():
    PORT = 8000
    DIRECTORY = "."

    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=DIRECTORY, **kwargs)

    web_dir = Path(__file__).parent.absolute()
    os.chdir(web_dir)

    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"Server started at http://localhost:{PORT}/dashboard.html")
        print("Press Ctrl+C to stop the server")
        
        # Open the dashboard in the default browser
        webbrowser.open(f"http://localhost:{PORT}/dashboard.html")
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")

if __name__ == "__main__":
    start_server()