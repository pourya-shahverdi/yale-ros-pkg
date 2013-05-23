import sys
import yaml

files = sys.argv
files.pop(0)

for f in files:
    print "---- checking file: " + f + " ----"
    with open(f, 'r') as infile:
        s = infile.read()
    actions = yaml.load(s)

    expressions = ["angry",
                   "frightened",
                   "puppy",
                   "sipping",
                   "confused",
                   "lovestruck",
                   "sad",
                   "tasting",
                   "disgusted",
                   "mischievous",
                   "surprised"]

    for action in actions:
        if not "type" in action.keys():
            print "missing type"
        elif action["type"] == "expression":
            if not "start" in action.keys():
                print "missing start"
            if not "id" in action.keys():
                print "missing id"
            elif not action["id"] in expressions:
                print "invalid id: " + str(action["id"])
    print "-------------------------------------"
