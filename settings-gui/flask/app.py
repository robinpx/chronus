from flask import Flask, request, url_for, render_template
from werkzeug.utils import redirect

app = Flask(__name__)

# Success
@app.route("/success")
def success():
    return render_template("success.html")


# Main page
@app.route("/", methods=["GET", "POST"])
def main():
    err = ""
    if request.method == "POST":
        if "save" in request.form.keys():
            new_settings = ""
            ind = 0
            for entry in request.form.items():
                if ind < 3:
                    if entry[1].replace(".", "", 1).isdigit():
                        if entry[1][0] == ".":
                            write_in = "0" + entry[1]
                        elif len(entry[1]) > 0:
                            write_in = entry[1]
                        # build text
                        if ind == 2:
                            new_settings += write_in
                        else:
                            new_settings += write_in + "\n"
                        ind += 1
                        flag = True
                    else:
                        flag = False
                        break
            if flag:
                text_file = open("dimensions.txt", "w")
                text_file.writelines(new_settings)
                text_file.close()
                print("Changes successful.")
                return redirect(url_for("success"))
            else:
                err = "No changes made. Decimals and numbers only."
                print(err)
        elif "reset" in request.form.keys():
            with open("dimensions.txt", "w") as text_file:
                text_file.write("80\n30\n1")  # added in default values

    with open("dimensions.txt", "r") as text_file:
        # reading current values from file
        content = text_file.read()
        setting_vals = content.strip().split("\n")
        if len(setting_vals) > 0:
            max_fr = setting_vals[0]
            min_fr = setting_vals[1]
            emb = setting_vals[2]
        return render_template(
            "index.html", max_frames=max_fr, min_frames=min_fr, embed=emb, error=err
        )


if __name__ == "__main__":
    app.run(debug=True)
