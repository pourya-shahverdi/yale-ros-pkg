import sys
import yaml

files = sys.argv
files.pop(0)

subs = {"afraid": "frightened",
        "blech":"disgusted",
        "idunno":"confused",
        "interest":"puppy",
        "mmhmmm":"puppy",
        "question":"confused",
        "wakeup":"surprised",
        "yes":"lovestruck",
        "anticipation":"puppy",
        "farted":"mischievous",
        "i_like_it": "lovestruck",
        "ilikeit":"lovestruck",
        "laugh1":"mischievous",
        "mph":"angry",
        "surprise":"surprised",
        "yay":"lovestruck",
        "yummm":"lovestruck",
        "bite":"tasting",
        "hungry":"puppy",
        "i_want_it":"puppy",
        "iwantit":"puppy",
        "meh":"angry",
        "no":"angry",
        "think":"confused",
        "yawn":"puppy"}
        
        





for f in files:
    print "Filtering file: " + f
    with open(f, 'r') as infile:
        s = infile.read()
    actions = yaml.load(s)

    for action in actions:
        if action["type"] == "motion":
            if action["id"] in subs.keys():
                action["id"] = subs[action["id"]]
            elif action["id"] not in subs.values():
                print "invalid id: " + str(action["id"])
            action["type"] = "expression"
    with open(f,'w') as outfile:
        yaml.dump(actions, outfile)
