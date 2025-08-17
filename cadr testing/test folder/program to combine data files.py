import json
filename = "sps30samples"
file_ending = ".json"
name_counter = 0
samples = []
while True:
    try:
        file = open(filename+str(name_counter)+file_ending, 'r')
        samples_batch = json.load(file)
        name_counter += 1
        samples = samples + samples_batch
    except:
        print("done?")
        #print(samples)
        print("number of samples apparently:",len(samples))
        with open("samplescombined.json", "w") as outfile:
            json.dump(samples, outfile)
        break
