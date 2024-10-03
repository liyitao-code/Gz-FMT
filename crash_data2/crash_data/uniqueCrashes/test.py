import re


def testRe(file):
    with open(file) as f:
        content = f.read()
        for line in content.splitlines():
            if line.startswith("Stack trace"):
                continue
            elif line.startswith("Segmentation fault"):
                continue
            #m = re.match(r'#\d\s+Object ".*?", at .*?, in (.*\(.*\))', line)
            #m = re.match(r'#\d\s+Object ".*?", at .*?, in (.*?\(.*?\)|.*)', line)
            # m = re.match(r'#\d\s+Object ".*?", at .*?, in (.*)', line)
            # if m:
            #     print(m.group(1))
            elif line.startswith("#"):
                items = line.split(" in ")
                if items[-1]:
                    print(items[-1])
            


if __name__ == "__main__":
    testRe("./uniqueCrashes/unique_crash5/2624/gz.err")