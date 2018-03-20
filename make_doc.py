from __future__ import print_function
import subprocess
import shutil
import os

DOCDIR = "doc"
APIDIR = os.path.join(DOCDIR,"api")

print("==================\nAPI-DOC-Generator\n==================")

#print("*** delete old API-Files ***")
#try:
#    shutil.rmtree(APIDIR)
#except:
#    print("delete APIDIR failed")

#print("*** start shinx-apidoc ***")
#subprocess.call(["sphinx-apidoc", "-e",  "-o", APIDIR, "camera"],shell=True)
print("*** make clean ***")
subprocess.call(["make.bat","clean"],cwd=DOCDIR, shell=True)
print("*** make html ***")
subprocess.call(["make.bat","html"],cwd=DOCDIR, shell=True)

print("*** finished ***")
