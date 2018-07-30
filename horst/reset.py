import sysv_ipc
from struct import *
mq = sysv_ipc.MessageQueue(1241, sysv_ipc.IPC_CREAT)
mq.remove()
