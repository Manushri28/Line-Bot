import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/manushri_kumbhakar/linebot_ws/install/linebot_follower'
