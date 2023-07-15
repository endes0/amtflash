# amtflash
Device driver for the AMT Flash (MPPS) v13 OBD Cable

# Usage
``` python
from amtflash import AMTFlash
device = AMTFlash()
print("Version: {}".format(device.get_version()))
print("Version: {}".format(device.get_version_str()))
´´´