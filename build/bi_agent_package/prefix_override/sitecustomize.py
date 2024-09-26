import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nachiketa/dup_auto_ass1/install/bi_agent_package'
