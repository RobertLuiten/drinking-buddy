from flask import Flask, send_from_directory, render_template, url_for

app = Flask(__name__)  

# @app.route("/")
# def root():
#     return 



@app.route('/')  
def root():  
    return render_template('main.html')

# @app.route('/static/<file>')
# def static(file):
#     return 