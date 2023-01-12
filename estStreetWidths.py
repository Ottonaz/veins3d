#!/usr/bin/env python3

"""
This script estimates the street width (i.e., distance between the buildings to
either side of the street) for all edges (which allow vehicles) in a SUMO road network.
The widths are added as parameters to the edges and the road network is written to a new file.
Furthermore, another set of polygons is computed which can be loaded in SUMO to
visualize the street widths.
"""

import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")
import sumolib
import sumolib.geomhelper as g
import rtree
import numpy as np
import lxml.etree as et



def calcMinDist(subseg, bb):
    minDist = 60.0 # we don't care for larger distances
    
    for p in polyTree.intersection((bb[0] - minDist, bb[1] - minDist, bb[2] + minDist, bb[3] + minDist), objects='raw'):
        for c in p.shape:
            dist = g.distancePointToPolygon(c, subseg)
            if dist < minDist:
                minDist = dist
    return minDist

def calcStreetWidth(sh, bb, maxseglen):
    dists = []
    for i in range(0, len(sh) - 1):
        s0 = sh[i]
        s1 = sh[i + 1]
        direc = g.norm(g.sub(s1, s0))
        seglen = g.length(g.sub(s1, s0))
        numsubs = 1
        if seglen > maxseglen:
            numsubs = int(np.ceil(seglen / maxseglen))
            seglen = seglen / numsubs
        for j in range(0, numsubs):
            subpt1 = g.add(s0, g.mul(direc, seglen * j))
            subpt2 = g.add(s0, g.mul(direc, seglen * (j + 1)))
            subseg = (subpt1, subpt2)
            dist = calcMinDist(subseg, bb)
            dists.append(dist)
    streetWidth = np.average(dists) * 2
    return streetWidth
            
def constructVisualization(sh, width):
    corners = []
    vbefore = ()
    for i in range(0, len(sh) - 1):
        s0 = sh[i]
        s1 = sh[i + 1]
        vi = g.norm(g.sub(s1, s0))
        if len(corners) == 0:
            ni = (vi[1], -vi[0])
            corners.append(g.add(s0, g.mul(ni, 0.5*width)))
            corners.append(g.sub(s0, g.mul(ni, 0.5*width)))
            vbefore = vi
        else:
            c1 = ()
            c2 = ()
            di = g.sub(vi, vbefore)
            if di == (0.0, 0.0):
                di = (vi[1], -vi[0])
                c1 = g.add(s0, g.mul(di, 0.5*width))
                c2 = g.sub(s0, g.mul(di, 0.5*width))
            else:
                angle = np.arctan2(vi[1], vi[0]) - np.arctan2(-vbefore[1], -vbefore[0])
                if angle < 0:
                    angle += 2*np.pi
                factor = np.sin(angle/2)
                if angle >= np.pi:
                    factor = -factor
                di = g.norm(di)
                c1 = g.add(s0, g.mul(di, 0.5*width/factor))
                c2 = g.sub(s0, g.mul(di, 0.5*width/factor))
                
            pos = int(len(corners)/2)
            corners.insert(pos, c1)
            corners.insert(pos + 1, c2)
            vbefore = vi
            
        if len(corners) != 0 and i == len(sh) - 2:
            ni = (vi[1], -vi[0])
            pos = int(len(corners)/2)
            corners.insert(pos, g.add(s1, g.mul(ni, 0.5*width)))
            corners.insert(pos + 1, g.sub(s1, g.mul(ni, 0.5*width)))
        
    return corners

def buildTree(polys):
    polyTree = rtree.index.Index()
    polyTree.interleaved = True
    for i, poly in enumerate(polys):
        polyTree.add(i, poly.getBoundingBox(), poly)
    return polyTree
    


if __name__ == '__main__':
    if len(sys.argv) != 4 and len(sys.argv) != 5:
        print("Usage:\n./estStreetWidths.py <original net.xml file> <polygon file> <new net.xml file> "\
              "[<new polygon file for visualizing street widths>]")
        sys.exit(1)
    netFile = sys.argv[1]
    polyFile = sys.argv[2]
    newNetFile = sys.argv[3]
    visualFile = None
    if len(sys.argv) == 5:
        visualFile = sys.argv[4]
    
    print("Loading road network and polygons...")
    net = sumolib.net.readNet(netFile)
    netWInt = sumolib.net.readNet(netFile, withInternal=True)
    allpolys = sumolib.shapes.polygon.read(polyFile)
    polys = [p for p in allpolys if "building" in p.type]
    edges = net.getEdges()
    nodes = net.getNodes()
    
    polyTree = buildTree(polys)
             
    streetWidths = {}
    
    numTotal = 0
    numRoads = 0
    sumWidth = 0.0
    print("Estimating street widths...")
    for e in edges:
        if numTotal%100 == 0:
            print(numTotal, "/", len(edges))
        numTotal += 1
        if not e.allows("private"):
            continue
        numRoads += 1
        sh = e.getRawShape()
        streetWidth = calcStreetWidth(sh, e.getBoundingBox(True), 10.0)
        
        streetWidths[e.getID()] = streetWidth    
        sumWidth += streetWidth
        
    print(numTotal, "/", numTotal)
    print("Estimated street width for", numRoads, "of", numTotal, "edges (which allow vehicles), avg was:", sumWidth/numRoads)
    
    numTotal = 0
    print("Determining street widths of internal edges...")
    for n in nodes:
        maxWidth = 0.0
        for e in (n.getOutgoing() + n.getOutgoing()):
            sw = streetWidths.get(e.getID(), 0.0)
            if sw > maxWidth:
                maxWidth = sw
        for e in netWInt.getNode(n.getID()).getIncoming():
            if e.getFunction() == "internal":
                streetWidths[e.getID()] = maxWidth
    
    print("Generating new net.xml file with street widths added as parameters...")
    parser = et.XMLParser(remove_blank_text=True)
    netXml = et.parse(netFile, parser)
    edgesXml = netXml.findall("edge")
    for e in edgesXml:
        edgeId = e.get("id")
        if edgeId in streetWidths:
            newParam = et.SubElement(e, "param")
            newParam.set("key", "streetWidth")
            newParam.set("value", str(streetWidths[e.get("id")]))
    netXml.write(newNetFile, pretty_print=True, encoding="UTF-8", xml_declaration=True)
    
    if visualFile != None:
        print("Generating polygons which can be used to visualize street widths...")
        with open(visualFile, "w") as myfile:
            myfile.write('<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">\n')
            polyXml = et.parse(polyFile, parser)
            loc = polyXml.find("location")
            myfile.write("  " + et.tostring(loc, encoding="UTF-8", pretty_print=True).decode("utf-8"))
            for eid in streetWidths:
                if not net.hasEdge(eid): # internal edges' widths are not visualized
                    continue
                e = net.getEdge(eid)
                sh = e.getRawShape()
                widthVisuals = constructVisualization(sh, streetWidths[eid])
            
                myfile.write('  <poly id="' + eid + 'Width" type="boundary" color="50,237,50" fill="0" layer="-1.00" shape="')
                for p in widthVisuals:
                    myfile.write(str(p[0]) + "," + str(p[1]))
                    myfile.write(" ")
                myfile.write(str(widthVisuals[0][0]) + "," + str(widthVisuals[0][1]) + '"/>\n')
            
            myfile.write('</additional>')
        
    print("Done.")