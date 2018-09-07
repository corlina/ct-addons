import subprocess
import zipfile
import os

try:
    os.remove('ct_addons.zip')
except:
    pass
subprocess.check_call('zip -r ct_addons.zip ct_addons', shell=True)

with zipfile.ZipFile('ct_addons.zip', 'a') as zf:
    zf.write('ct_addons/__main__.py', '__main__.py')
