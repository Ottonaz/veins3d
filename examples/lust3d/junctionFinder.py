import xml.etree.ElementTree as ET
import math

xlimit = (4870,6476)
ylimit = (6467,7234)

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def parse_net_xml(file_path, tolerance=1000.0):
    tree = ET.parse(file_path)
    root = tree.getroot()

    crossings = root.findall(".//junction")
    crossings_count = {}

    minx = 10000000
    miny = 10000000
    for crossing in crossings:
        x_pos = float(crossing.attrib['x'])
        y_pos = float(crossing.attrib['y'])

        if(x_pos > xlimit[0] and x_pos < xlimit[1] and y_pos > ylimit[0] and y_pos < ylimit[1] ):
            if (int(x_pos / tolerance)*tolerance, int(y_pos / tolerance)*tolerance) not in crossings_count.keys():
                crossings_count[(int(x_pos / tolerance)*tolerance, int(y_pos / tolerance)*tolerance)]= [(x_pos,y_pos)]     
            else:
                crossings_count[(int(x_pos / tolerance)*tolerance, int(y_pos / tolerance)*tolerance)].append((x_pos,y_pos))    
        
            if minx > x_pos:
                minx = x_pos
            if miny > y_pos:
                miny = y_pos

    n = 0
    file = open("rsus.txt","w")
    for key, value  in crossings_count.items():
        if len(value) > 1:
            print(f"*.rsu[{n}].mobility.x = { int(value[0][0]) +26 }")
            print(f"*.rsu[{n}].mobility.y = { 11480 - int(value[0][1])}")
            file.write(f"*.rsu[{n}].mobility.x = {11480 - int(value[0][0])}\n")
            file.write(f"*.rsu[{n}].mobility.y = {int(value[0][1]) + 26}\n")
            
            n += 1
    print("crossing found: ", n)

if __name__ == '__main__':
    file_path = "lust3d.net.xml"  # Replace with the actual path to your .net.xml file
    size = input("grid size: ")
    parse_net_xml(file_path,  int(size))
    
    

