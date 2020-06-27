#! /usr/bin/python
import os
from os.path import expanduser
import shutil

class DirCleanup:
    """
    This class deletes unused directories created for BHG. The assumption
    is that if the directory has less than 5 files in it, then it should
    be deleted
    """
    
    def __init__(self):
        self.dirs_to_delete = set()  
        self.data_dir = '/home/user1/Data/'      

    def find_dirs_to_delete(self, path):
        """ 
        Finds the unused directories adds them to a set. Unused directories are
        defined as those with less than 5 files/direcotries.
        """
        self.data_dir = path
        for root, dirs, files in os.walk(path):
            for folder in dirs:
                check_dir = os.path.join(root, folder)
                num_files = sum([len(files) for r, d, files in os.walk(check_dir)])
                if (num_files < 5): 
                    self.dirs_to_delete.add(check_dir)
                #print(check_dir,num_files)
            break
        #print(len(list_subfolders_with_paths))
        
    def repair_symlink(self, latest_dir):
        """ 
        Creates a symlink to the latest created folder. This is needed for the cases where
        the symlink is pointed to a directory that is not used. When that directory is deleted
        the symlink is broken and is repaired by pointing it to the most current used directory.
        """   
        os.unlink(latest_dir)
        files = os.listdir(self.data_dir)
        files.sort(reverse=True)
        print(self.dirs_to_delete)
        for f in files:
            logfile_dir = self.data_dir + f
            if logfile_dir not in self.dirs_to_delete:
                print("++++++ REPAIRING SYMLINK %s +++++++" % logfile_dir)
                os.symlink(logfile_dir, latest_dir) 
                break
        
    def delete_folders(self):     
        """ Deletes all unused directories except the symlink 'latest' which is repaired."""   
        for folder in self.dirs_to_delete:
            print folder
            if "latest" in folder: # link to the last useful directory
                self.repair_symlink(folder) 
            else: 
                print("++++++ DELETING UNUSED FOLDER %s +++++++" % folder)
                shutil.rmtree(folder)
                  
    def get_dirs_to_delete(self):
        """ Return a list of all unused directories """   
        return self.dirs_to_delete
        
    def move_bagfile(self, path): #TODO this does not work. Need to fix it.
        """ Moves the latest bagfile to the latest directory """
        # this is not working as expected
        pass
#        files = os.listdir(path)
#        files.sort(reverse=True)
#        for item in files:
#            if "bag" in item:
#                abspath = path + item
#                destpath = path + "latest/" + item
#                shutil.move(abspath, destpath)
 
if __name__ == '__main__':
    # This is for testing and example on how to use
    dc = DirCleanup()    
    home = expanduser("~")
    rootDir = home + "/Data/"
    dc.move_bagfile(rootDir)
    dc.find_dirs_to_delete(rootDir)
    dc.delete_folders()
    print("Deleting the following: " , dc.get_dirs_to_delete())




