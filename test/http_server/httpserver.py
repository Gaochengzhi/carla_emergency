import http.server
import socketserver
import threading
import requests
import uuid
import pickle
from io import BytesIO

class SimpleHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    message = None

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "application/octet-stream")
        self.end_headers()
        if SimpleHTTPRequestHandler.message:
            self.wfile.write(SimpleHTTPRequestHandler.message)
        else:
            self.wfile.write(pickle.dumps(None))

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        SimpleHTTPRequestHandler.message = self.rfile.read(content_length)
        self.send_response(200)
        self.end_headers()

class CommuniAgent:
    def __init__(self, name: str):
        self.type = name
        self.id = uuid.uuid1()
        self.httpd = None
        self.server_thread = None

    def start_server(self, port):
        handler = SimpleHTTPRequestHandler
        self.httpd = socketserver.TCPServer(("", port), handler)
        self.httpd.allow_reuse_address = True
        self.server_thread = threading.Thread(target=self.httpd.serve_forever)
        self.server_thread.start()
        print(f"Server started on port {port}")

    def stop_server(self):
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()
            self.server_thread.join()
            self.httpd = None

    def send_message(self, url, obj):
        serialized_data = pickle.dumps(obj)
        requests.post(url, data=serialized_data)

    def get_message(self, url):
        response = requests.get(url)
        return pickle.loads(response.content)

    def close(self):
        self.stop_server()

# 使用示例
# 创建并启动服务器
agent = CommuniAgent("TestAgent")
agent.start_server(8000)

# 发送和接收对象
test_obj = {'key': 'value'}
agent.send_message("http://localhost:8000", test_obj)
received_obj = agent.get_message("http://localhost:8000")
print("Received object:", received_obj)

# 关闭服务器
agent.close()
