import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/npd/Desktop/amr_bot/src/amr_controller/install/amr_controller'
