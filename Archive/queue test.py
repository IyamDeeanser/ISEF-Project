#queue test
import queue
q = queue.Queue()
q.put(1)
while True:
    hi = q.get()
    print(hi)