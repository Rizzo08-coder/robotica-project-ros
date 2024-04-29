from flask import Flask

app = Flask(__name__)

#API route
@app.route("/list")
def hello_world():
    return {"list" : ["1","2","3"] }

if __name__ == "__main__":
    app.run(debug=True, host="localhost")