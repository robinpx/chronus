with open("./flask/dimensions.txt", "r") as text_file:
    content = text_file.read()
    content = content.strip().split("\n")
    settings = dict()
    for line in content:
        k,v=line.split(",")
        print(k,v)
        if k != "re_embed":
            settings[k]=float(v)
        else:
            if v == "on":
                settings[k]=True
            else:
                settings[k]=False
    print(settings)
    # to decide length trajectory roomba, num frame between a min and a max:
    MAX_FRAMES = settings["max_frames"]
    MIN_FRAMES = settings["min_frames"]
    # sensitivity threshold to judge if 3 points are almost aligned
    COLINEARITY_THRESHOLD = settings["colinear"]
    # Approximate room radius, to scale the embedding
    ROOM_RADIUS = settings["room_radius"]
    # When will listen to new message from Roomba
    INTERVAL_LISTEN = settings["interval_listen"] #NB: Could increase if want less point
    # roomba sattelite threshold
    SAT_THRESHOLD = settings["sat_threshold"]
    # number of initial frames to warm up roomba, before word reading etc.
    WARM_UP=settings["warm_up"]
    # maximum number of concept roomba is saying
    MAX_WORDS_PER_EVENT=settings["max_words"]
    # If redefine gpt2 embeddings
    REDEFINE_EMBEDDINGS=settings["re_embed"]
