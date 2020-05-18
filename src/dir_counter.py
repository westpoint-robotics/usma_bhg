#! /usr/bin/python
import os
from os.path import expanduser
import shutil

class DirCounter:
    """
    This class deletes unused directories created for BHG. The assumption
    is that if the directory has less than 5 files in it, then it should
    be deleted
    """
    
    def __init__(self):
        self.dir_to_count = ""  
        self.num_files = 0      

    def count_dir(self, path):
        """ Finds the unused directories adds them to a set """
        self.dir_to_count = path
        dir_list = os.listdir(path)
        self.num_files = len(dir_list)
        list_subfolders_with_paths = []
        for root, dirs, files in os.walk(path):
            for dir in dirs:
                print(os.path.join(root, dir), len(os.listdir(os.path.join(root, dir))))
                

if __name__ == '__main__':
    # This is for testing and example on how to use
    dc = DirCounter()    
    home = expanduser("~")
    dir2c = home + "/Data/latest"
    dc.count_dir(dir2c)
    print (dc.num_files)




