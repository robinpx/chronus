from flask import Flask, render_template
import json
import selenium_driver

app = Flask(__name__)

driver = selenium_driver.SeleniumDriver()


@app.route("/")
def main():
    return render_template("index.html")


@app.route("/image", methods=["GET"])
def get_data():
    flag = driver.write_to_body("Hello, world")
    if flag:
        image_data = driver.get_submit_captcha()
        response = app.response_class(
            response=json.dumps(image_data), mimetype="application/json"
        )
    else:
        error = {"status": 404, "message": "Requested resource not found"}
        response = app.response_class(
            response=json.dumps(error), mimetype="application/json"
        )
    return response


if __name__ == "__main__":
    app.run(debug=True)
