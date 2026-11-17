import os,pty,time
master,slave=pty.openpty();path=os.ttyname(slave)
os.chown(path,1000,1000);os.chmod(path,0o660)
try:os.unlink('/dev/ttyS2')
except FileNotFoundError:pass
os.symlink(path,'/dev/ttyS2');os.close(slave)
while True:time.sleep(60)
