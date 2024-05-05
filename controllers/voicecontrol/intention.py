import os
import time
import fcntl

file_lock_path = '/tmp/webots/intention.lock'
file_path = '/tmp/webots/intention'

def acquire_lock():
    lockfile = open(file_lock_path, "w")
    fcntl.flock(lockfile, fcntl.LOCK_EX)

def release_lock():
    lockfile = open(file_lock_path, "w")
    fcntl.flock(lockfile, fcntl.LOCK_UN)

def read_file():
    acquire_lock()
    with open(file_path, 'r+') as f:
        data = f.read()
        f.truncate(0) # reset
    release_lock()

    return data.strip()

def write_file(text):
    acquire_lock()
    with open(file_path, 'r+') as f:
        f.truncate(0) # reset
        f.write(text)
    release_lock()