from flask import Flask, request, url_for, render_template
from werkzeug.utils import redirect

app = Flask(__name__)

def get_settings_dict():
    with open("dimensions.txt", "r") as text_file:
        # reading current values from file
        content = text_file.read()
        setting_lines = content.strip().split("\n")
        settings=dict()
        for line in setting_lines:
            k,v=line.split(",")
            settings[k]=v
    return settings

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
            new_settings = dict()
            ind = 0
            flag = True
            old_settings=get_settings_dict()
            # checking for values entered in text boxes
            for entry in request.form.items():
                k,v=entry[0],entry[1]
                if k != "save":
                    # print(k,v)
                    if len(v) > 0 and v.replace(".", "", 1).isdecimal():
                        if k != "re_embed":
                            if v[0] == ".":
                                v = "0" + v # write as appropriate decimal
                        # place in dict
                        new_settings[k]=v
                ind+=1

            # checking for re_embed values (not detected in request.form.items())
            v=request.form.getlist("re_embed")
            if len(v) == 0: # not valid entry for re_embed
                v.append("off")
                new_settings["re_embed"]=v[0]
        
            if len(new_settings.keys()) > 0:
                old_settings=get_settings_dict()
                for k,v in new_settings.items():
                    old_settings[k]=v
                with open("dimensions.txt","w") as text_file:
                    for k,v in old_settings.items():
                        text_file.write(k+","+v+"\n")
                print("Changes successful.")
                return redirect(url_for("success"))
            else:
                err = "Decimals and numbers only."
                print(err)
        
        elif "reset" in request.form.keys():
            with open("dimensions.txt", "w") as text_file:
                with open("default.txt", "r") as default:
                    content=default.read()
                    text_file.write(content)  # added in default values
                print("Changes successful.")
                return redirect(url_for("success"))
        
        
    settings=get_settings_dict()
    return render_template(
            "index.html", 
            room_radius=settings["room_radius"], 
            sat_threshold=settings["sat_threshold"], 
            max_frames=settings["max_frames"], 
            min_frames=settings["min_frames"], 
            colinear=settings["colinear"], 
            interval_listen=settings["interval_listen"], 
            warm_up=settings["warm_up"], 
            max_words=settings["max_words"], 
            re_embed=settings["re_embed"],
            error=err,
            error_len=len(err)
    )

if __name__ == "__main__":
    app.run(debug=True)
