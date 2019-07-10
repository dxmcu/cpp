#!/usr/bin/python3

import os

if __name__ == '__main__':
  cur_dir = os.getcwd()
  files = os.listdir(cur_dir)
  for file in files:
    if file.endswith('.h') or file.endswith('.cpp'):
      cmd = 'iconv -f gbk -t utf-8 '
      cmd = cmd + file
      sub_dir = ' > src/'
      cmd = cmd + sub_dir + file
      os.system(cmd)
  
