#!/usr/bin/python3

from google_images_search import GoogleImagesSearch
import os

key = os.getenv("IMAGES_SEARCH_KEY")
cx = os.getenv("IMAGES_SEARCH_CX")

gis = GoogleImagesSearch(key, cx)

def find_image(query):
    
    if os.path.isfile("./images/image.jpg"):
        os.remove("./images/image.jpg")
        print("Deleted old image")

    try:
        _search_params = {
            'q': query,
            'num': 1,
        }

        gis.search(search_params=_search_params, path_to_dir='./images', custom_image_name='image')
        print("Downloaded new image")
    
    except:
        print("New image download failed. Proceeding without.")
