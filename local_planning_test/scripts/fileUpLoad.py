import subprocess
import signal
import os
import sys
import paramiko
def fileUpLoad(filename,host_ip,username,password,remotepath):
    path = os.path.split(os.path.realpath(__file__))[0]
    path += filename
    ssh_port = 22
    transport = paramiko.Transport((host_ip, ssh_port))
    transport.connect(username=username, password=password)
    sftp = paramiko.SFTPClient.from_transport(transport)
    sftp.put(path, remotepath)
    transport.close()
    return True
def fileDownLoad(filename,host_ip,username,password,remotepath):
    localPath = os.path.split(os.path.realpath(__file__))[0]
    localPath += filename
    ssh_port = 22
    transport = paramiko.Transport((host_ip, ssh_port))
    transport.connect(username=username, password=password)
    sftp = paramiko.SFTPClient.from_transport(transport)
    sftp.get(remotepath,localPath)
    transport.close()

#if __name__=="__main__":
    #fileUpLoad("/test.txt")
    #fileDownLoad(filename='/test.txt',host_ip='192.168.31.242',username='zzy',
                 #password='1',remotepath='/home/zzy/hxy/test123.txt')