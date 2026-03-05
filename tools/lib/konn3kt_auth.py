#!/usr/bin/env python3
"""
konn3kt_auth.py - Authentication helper for cabana to access konn3kt routes

This script helps you authenticate with konn3kt and saves your auth token
so cabana can access your routes.

Usage:
  python konn3kt_auth.py

The script will:
1. Open your browser to log in via GitHub OAuth
2. Receive the JWT token from konn3kt
3. Save it to ~/.comma/auth.json for cabana to use
"""

import http.server
import json
import os
import socketserver
import sys
import urllib.parse
import webbrowser
from pathlib import Path

API_BASE = os.getenv("API_HOST", "https://api-konn3kt.teallvbs.xyz")
REDIRECT_PORT = 3847


class OAuthCallbackHandler(http.server.SimpleHTTPRequestHandler):
  token_received = None

  def do_GET(self):
    parsed = urllib.parse.urlparse(self.path)
    params = urllib.parse.parse_qs(parsed.query)

    if "token" in params:
      OAuthCallbackHandler.token_received = params["token"][0]

      self.send_response(200)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      self.wfile.write(b"""
            <html>
            <head><title>Authentication Successful</title></head>
            <body style="font-family: Arial; text-align: center; padding: 50px;">
                <h1>Authentication Successful!</h1>
                <p>You can close this window and return to your terminal.</p>
                <p>Your auth token has been saved to ~/.comma/auth.json</p>
            </body>
            </html>
            """)
    else:
      self.send_response(400)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      self.wfile.write(b"""
            <html>
            <head><title>Authentication Failed</title></head>
            <body style="font-family: Arial; text-align: center; padding: 50px;">
                <h1>Authentication Failed</h1>
                <p>No token received. Please try again.</p>
            </body>
            </html>
            """)

  def log_message(self, format, *args):
    pass


def save_token(token):
  auth_dir = Path.home() / ".comma"
  auth_dir.mkdir(exist_ok=True)

  auth_file = auth_dir / "auth.json"
  auth_data = {
    "access_token": token,
    "token_type": "JWT",
  }

  with open(auth_file, "w") as f:
    json.dump(auth_data, f, indent=2)

  auth_file.chmod(0o600)

  print(f"✓ Auth token saved to {auth_file}")


def main():
  print("=" * 60)
  print("konn3kt Authentication Helper")
  print("=" * 60)
  print()
  print(f"API Base: {API_BASE}")
  print(f"Callback Port: {REDIRECT_PORT}")
  print()

  handler = OAuthCallbackHandler

  try:
    with socketserver.TCPServer(("127.0.0.1", REDIRECT_PORT), handler) as httpd:
      oauth_url = f"{API_BASE}/v2/auth?redirect_after=http://127.0.0.1:{REDIRECT_PORT}/callback"

      print("Opening browser for GitHub OAuth login...")
      print("If the browser doesn't open, visit this URL:")
      print(f"  {oauth_url}")
      print()
      print("Waiting for authentication...")

      webbrowser.open(oauth_url)

      while handler.token_received is None:
        httpd.handle_request()

      print()
      save_token(handler.token_received)
      print()
      print("=" * 60)
      print("Authentication successful!")
      print("=" * 60)
      print()
      print("You can now use cabana to view your konn3kt routes:")
      print("  1. Open cabana")
      print("  2. Go to File > Open Remote Route")
      print("  3. Select your device and route")
      print()

  except KeyboardInterrupt:
    print("\n\nAuthentication cancelled.")
    sys.exit(1)
  except OSError as e:
    if e.errno == 98:
      print(f"\nError: Port {REDIRECT_PORT} is already in use.")
      print("Please close any other applications using this port and try again.")
      sys.exit(1)
    else:
      raise


if __name__ == "__main__":
  main()
