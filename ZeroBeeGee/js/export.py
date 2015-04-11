import json
import sys
import os

with open('visnav_edx_skulpt-stdlib.json')  as st:
    data = json.load(st)
    for fn, content in data['files'].items():
        dr = os.path.dirname(fn)
        directory = os.path.join("visnav_edx_skulpt-stdlib.src", dr)        
        if not os.path.exists(directory):
            os.makedirs(directory)
        with open("visnav_edx_skulpt-stdlib.src/{0}".format(fn), 'w') as outstream:
            outstream.writelines(content)
