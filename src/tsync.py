from threading import Thread

global val

class Mutex:
    def __init__(self, val):
        self.resource = val
        self.locked = False

    def acquire(self):
        while (self.locked):
            pass

        self.locked = True
    
    def release(self):
        self.locked = False
    
    def set(self, val):
        self.resource = val
    
    def get(self):
        return self.resource

class DoStuff:
    def __init__(self):
        self.running = True
    
    def terminate(self):
        self.running = False
    
    def run(self):
        global val
        for i in range(0,5000):
            if self.running:
                val.acquire()
                val.set(val.get() + 1)
                val.release()


def main():
    global val
    val = Mutex(0)
    
    t1 = Thread(target=DoStuff().run)
    t2 = Thread(target=DoStuff().run)
    
    print("Starting threads")
    t1.start()
    t2.start()
    
    print("Waiting for threads... ")
    t1.join()
    t2.join()

    print ("Threads have finished; ")
    print (val.get())

if __name__ == '__main__':
    main()
