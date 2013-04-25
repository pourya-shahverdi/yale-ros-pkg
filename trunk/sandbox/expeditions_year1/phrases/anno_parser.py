#!/usr/bin/env python

#Elaine Short
#Expeditions Year 1
#Annosoft file parser
import sys
import yaml

def main():
    if not len(sys.argv) == 4:
        print "Usage: anno_parser.py [list file] [target file] [data dir]"
        sys.exit()

    data = {}
    data_dir = sys.argv[3]

    vis_transl = {"AA":"AA_AH",
                  "AE":"EH_AE_AY",
                  "AH":"AA_AH",
                  "AO":"AO_AW",
                  "AW":"AO_AW",
                  "AY":"EH_AE_AY",
                  "b":"M_B_P",
                  "CH":"CH_SH_ZH",
                  "d":"N_NG_D_Z",
                  "DH":"N_NG_D_Z",
                  "EH":"EH_AE_AY",
                  "ER":"R_ER",
                  "EY":"EY",
                  "f":"M_B_P",
                  "g":"N_NG_D_Z",
                  "h":"AA_AH",
                  "IH":"EH_AE_AY",
                  "IY":"EY",
                  "j":"CH_SH_ZH",
                  "k":"N_NG_D_Z",
                  "l":"L",
                  "m":"M_B_P",
                  "n":"N_NG_D_Z",
                  "NG":"N_NG_D_Z",
                  "OW":"AO_AW",
                  "OY":"AO_AW",
                  "p":"EY",
                  "r":"R_ER",
                  "s":"N_NG_D_Z",
                  "SH":"CH_SH_ZH",
                  "t":"N_NG_D_Z",
                  "TH":"N_NG_D_Z",
                  "UH":"AO_AW",
                  "UW":"AO_AW",
                  "v":"M_B_P",
                  "w":"CH_SH_ZH",
                  "x":"IDLE",
                  "y":"EY",
                  "z":"N_NG_D_Z",
                  "ZH":"CH_SH_ZH",
                  "x":"IDLE"}
                  

    with open(sys.argv[1], 'r') as f_in:
            for line in f_in:
                name =  line.strip().split()[0]
                try:
                    with open(data_dir + "/" + name + ".txt") as textfile:
                        text = textfile.read()
                except IOError:
                    print "Error opening textfile"
                    text = "<" + name + " missing text>"
                visemes = []
                with open(data_dir +"/"+ name + ".anno") as annofile:
                    it = 0
                    for item in annofile:
                        viseme = item.strip().split()
                        if viseme[0] == "phn":
                            if viseme[4] in vis_transl:
                                t = vis_transl[viseme[4]]
                            else:
                                print "Warning: unknown viseme"
                                continue
                            visemes.append({"type": "viseme",
                                                           "id":t,
                                                           "start":float(viseme[1])/1000,
                                                           "end":float(viseme[2])/1000})
                            it = it + 1
                actions = []
                if len(line.strip().split()) > 1:
                    with open(data_dir + "/" + line.strip().split()[1], 'r') as actfile:
                        s = actfile.read()
                        actions = yaml.load(s)
                behavs = []
                behavs = actions + visemes
                data[name] = {"text": text,
                              "file": data_dir + "/" + name + ".wav",
                              "actions": behavs}
                
                    

    with open(sys.argv[2], 'w') as f_out:
        f_out.write("# Auto-generated phrase file for the dragonbot\n")
        f_out.write("# Generated from file: " + sys.argv[1] + "\n\n")
        yaml.dump(data,f_out,default_flow_style = False)
            

if __name__ == '__main__':
    main()
