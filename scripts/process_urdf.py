#!/usr/bin/python

import sys
import argparse
import rospkg
import fileinput


parser = argparse.ArgumentParser(description='Filters the daes of an URDF to rotate to Y up direction')
parser.add_argument('urdf_path', type=str)
args = parser.parse_args()

class ChangeDAE:
  def __init__(self, filename):
    self.filename = filename
    self.openURDF()
    pass

  def openURDF(self):
    with open(self.filename) as infile:
      for line in enumerate(infile,1):
        if(str(line).find("mesh"))!=-1 and str(line).find("dae")!=-1:
          daeFile = ((str(line).split("//")).pop()).split('"')[0]
          self.openDAE(str(daeFile))
    pass

  def openDAE(self, filename):
    rospack = rospkg.RosPack()
    name = filename.split("/")[:]
    stri = rospack.get_path(name[0])
    fullpath = stri
    for i in name[1: ]:
      fullpath += "/"+str(i)
    #print fullpath
    self.changeInDae(fullpath)
    pass

  def changeInDae(self, filename):
    for line in fileinput.input(filename, inplace=1):
      line = line.replace("<up_axis>Z_UP</up_axis>", "<up_axis>Y_UP</up_axis>")
      print line,
    pass



def main():
    print "I will rotate every DAE to Y up direction"
    ChangeDAE(args.urdf_path)
    print "DONE"
    
    

if __name__ == '__main__':
   main()
