class Node:
    all = []

    def __init__(self, name, stepSize):
        Node.all.append(self)
        self.name = name
        self.stepSize = stepSize
        self.connections = []


class Connection:
    all = []

    def __init__(self, pre, post, latency, maxBuffer = 1000):
        Connection.all.append(self)
        pre.connections.append(self)
        self.name = '%s->%s' % (pre.name, post.name)
        self.pre = pre
        self.post = post
        self.latency = latency
        self.allowedBuffer = maxBuffer


# Build a test graph

a = Node('a', 13)
b = Node('b', 3)
c = Node('c', 5)
d = Node('d', 7)
e = Node('e', 11)

Connection(a, b, 0)
Connection(b, c, 0)
Connection(c, d, 8)
Connection(d, b, 7)
Connection(c, e, 50)
Connection(e, b, 50)
Connection(d, e, 40)
Connection(e, d, 60)


# Search for loops

def depthFirst(x, path):
    if x.inPath:
        # Search path to detect beginning of loop
        for cIndex, c in enumerate(path):
            if c.pre is x:
                loop = path[cIndex:]

        print('Found loop', [c.name for c in loop])

        # Compute how much headroom we have for buffering
        totalDelay = 0
        for c in loop:
            totalDelay += c.latency - c.pre.stepSize

        # If negative we will not be able to make it in time
        # around the loop even without any extra buffering
        if totalDelay < 0:
            print('Too short latency (%g) around loop: ' % -totalDelay)
            print([c.name for c in loop])
            return False

        # Distribute totalDelay as allowed buffering uniformly over loop
        # (we could do better by considering constraints form other loops)
        bufDelay = totalDelay / len(loop)
        for c in loop:
            c.allowedBuffer = min(c.allowedBuffer,
                                  bufDelay // c.pre.stepSize)

        # Calculate and print out debug info
        print('Distributing delays',
              [(c.name,
                bufDelay // c.pre.stepSize,
                c.allowedBuffer)
               for c in loop])

        totalBufferedDelay = 0
        for c in loop:
            totalBufferedDelay += c.latency - c.pre.stepSize*c.allowedBuffer
        print('Total buffered delay', totalBufferedDelay)

        return True

    # Mark as processed (remove from main loop forest)
    x.visited = True
    x.inPath = True

    # Recurse in depth-first order
    for c in x.connections:
        depthFirst(c.post, path+[c])

    x.inPath = False


def findLoops(nodeList):
    for x in nodeList:
        x.visited = False
        x.inPath = False

    # Do depth-first traversal of forest
    for x in nodeList:
        if not x.visited:
            depthFirst(x, [])


findLoops(Node.all)

for c in Connection.all:
    print('%s %d'%(c.name, c.allowedBuffer))


def simulate(c):
    s = 0
    r = 0

    while s < 100:
        sStart = s

        # Advance receive time as much as possible
        # still ensuring that oldest data arrives in time
        while r + c.post.stepSize <= s + c.latency:
            r += c.post.stepSize

        # Advance send time to match receive time
        bCount = 0
        while r + c.post.stepSize >= s + c.latency:
            s += c.pre.stepSize
            bCount += 1

        # Advance send time according to precalculated buffer
        if bCount < c.allowedBuffer:
            s += c.pre.stepSize * (c.allowedBuffer - bCount)

        print('%s data from %d-%d, sent at %d, received at %d, latency %d (%d)'
              % (c.name, sStart, s, s, r, r-sStart, c.latency))


def simulate_sender(c):
    s = 0
    r = 0
    bCount = 0

    while s < 100:
        sStart = s

        # Advance receive time as much as possible
        # still ensuring that oldest data arrives in time
        while r + c.post.stepSize <= s + c.latency:
            r += c.post.stepSize

        # Advance send time to match receive time
        if r + c.post.stepSize < s + c.latency and bCount >= c.allowedBuffer:
            print('%s data from %d-%d, sent at %d, received at %d, latency %d (%d)'
                  % (c.name, sStart, s, s, r, r-sStart, c.latency))
            bCount = 0
            
        s += c.pre.stepSize
        bCount += 1


for c in Connection.all:
    simulate(c)
