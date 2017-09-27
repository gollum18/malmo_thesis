import constants, re

def is_digit(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

if __name__ == '__main__':
    i = 0
    for filename in constants.mission_xml:
        output = open(constants.mission_txt[i], 'w')
        reader = open(filename, 'r')
        output.write("// Obstacles\n")
        for line in reader.readlines():
            if 'DrawCuboid' not in line and 'DrawBlock' not in line:
                continue
            s = re.findall(r"['\"](.*?)['\"]", line)
            if 'gold_block' in s:
                continue
            out = ""
            if 'DrawCuboid' in line:
                output.write("{0} {1} {2} {3} {4} {5}\n".format(
                    s[0], s[3], s[1], s[4], s[2], s[5]
                ))
            elif 'DrawBlock' in line:
                output.write("{0} {1} {2} {3} {4} {5}\n".format(
                    s[0], s[0], s[1], s[1], s[2], s[2]
                ))
        reader.close()
        output.flush()
        output.close()
        i += 1