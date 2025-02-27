import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/npd/Desktop/amr_bot/src/install/amr_controller'
