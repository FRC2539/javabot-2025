import imgkit
import os
import sys

my_path = os.path.abspath("background.png")

value = sys.argv[1]

f = f"""
<style>
    @import url('https://fonts.googleapis.com/css2?family=Source+Code+Pro:ital,wght@0,700;1,700&display=swap');
    * {{
        margin: 0px;
        font-family: "Source Code Pro";
        font-optical-sizing: auto;
        font-weight: 700;
        font-style: normal;
    }}
</style>
<div style="width:300px">
    <img width="300px" src="{my_path}">
    <div style="font-size:65px; position: absolute; top: 20px; left: 0px; width: 300px;">
        <div style="margin-left: auto; margin-right: auto; width: min; text-align: center;">{value}</div>
</div>
"""

options = {
    "enable-local-file-access": "",
    "width": "300",
    "height": "163"
  }
imgkit.from_string(f, 'build/loc.png', options)
