import os
import json

g = os.walk('visnav_edx_skulpt-stdlib.src')
result = {}

for item in g:
    dir = item[0]
    files = item[2]

    for f in files:
        fname = os.path.join(dir, f)
        result[fname[len('visnav_edx_skulpt-stdlib.js/')+1:]] = open(fname, 'r').read()

with open("visnav_edx_skulpt-stdlib.js", "w") as outstream:
    outstream.write("Sk.builtinFiles=")
    json.dump({ "files": result }, outstream)
