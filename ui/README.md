# Setting up the server:
## Set up the venv
Make sure you are in the /ui/server folder!
```
python3 -m venv .venv
. .venv/bin/activate
```
## Install required libraries (yes Flask is capitalized)
```
pip install Flask
pip install roslibpy
pip install flask-socketio
```
## Run the server
```
python3 server.py
```
## Activate tunneling 
```
ssh -R 80:localhost:5000 serveo.net
```