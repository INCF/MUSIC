######################################################################

class Msg(object):
    def __init__(self, rank, step, time, when):
        self.rank = rank
        self.step = step
        self.time = time
        self.when = when

    def __str__(self):
        return "{}, {}, {}, {}". \
            format(self.rank,
                   self.step,
                   self.time,
                   self.when)

######################################################################
